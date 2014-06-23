/*
 * CUDA source file for ImplicitSurface class.
 */

#define check(expr, msg) { \
	cudaError_t err; \
	err = expr; \
	if (err != cudaSuccess) { std::cerr << "CUDA ERROR (" << cudaGetErrorString(err) << "): " << msg << std::endl; } \
	}
	
#define checkThrust(expr, msg) { \
	try { \
	expr; \
	} catch(thrust::system_error &e) { \
		std::cerr << "THRUST ERROR (" << e.what() << "): " << msg << std::endl; \
	} \
	}

#include <util/interface/Util.hpp>
#include <util/interface/ImplicitSurface.hpp>
#include <glm/interface/gtc/type_ptr.hpp>

#include <util/interface/cutil_math.h>

#include <thrust/device_ptr.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>

#include <limits.h>
#include <float.h>

// constant data
__constant__ float3 c_bboxMin;
__constant__ float3 c_bboxMax;

// textures
texture<float4, 3, cudaReadModeElementType> tex_voxels;

inline __device__ glm::vec3 make_vec3(const float3& a) {
	return glm::vec3(a.x, a.y, a.z);
}

__device__
bool clipRayAgainstAABB(const glm::vec3& org, const glm::vec3& dir, float& tnear, float& tfar) {
	glm::vec3 T_1, T_2;
	tnear = -FLT_MAX;
	tfar = FLT_MAX;
	
	glm::vec3 min(make_vec3(c_bboxMin));
	glm::vec3 max(make_vec3(c_bboxMax));
	
	for (int i = 0; i < 3; i++){
		if (dir[i] == 0){
			if ((org[i] < min[i]) || (org[i] > max[i])) {
				return false;
			}
		} else {
			T_1[i] = (min[i] - org[i]) / dir[i];
			T_2[i] = (max[i] - org[i]) / dir[i];

			if (T_1[i] > T_2[i]) {
				glm::vec3 tmp = T_1;
				T_1 = T_2;
				T_2 = tmp;
			}
			if (T_1[i] > tnear) {
				tnear = T_1[i];
			}
			if (T_2[i] < tfar) {
				tfar = T_2[i];
			}
			if ( (tnear > tfar) || (tfar < 0.0f) || (fabsf(tfar - tnear) < 0.0001f) ) {
				return false;
			}
		}
	}
	
	return true;
}

inline __device__
void evaluateGrid(const glm::vec3& target, float& value, glm::vec3& normal) {
	glm::vec3 coords = (target - make_vec3(c_bboxMin)) / (make_vec3(c_bboxMax) - make_vec3(c_bboxMin));
	float4 interp = tex3D(tex_voxels, coords.x, coords.y, coords.z);
	value = interp.w;
	normal = glm::vec3(interp.x, interp.y, interp.z);
}

__global__
void raymarchKernel(glm::vec3 org, glm::vec3* d_rays, PointNormalPair* d_output, unsigned n, unsigned steps) {
	unsigned idx = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (idx > n) {
		return;
	}
	
	glm::vec3 dir = d_rays[idx];
	
	glm::vec3 hit, normal;
	float tnear, tfar;
	
	bool intersects = clipRayAgainstAABB(org, dir, tnear, tfar);
	if (!intersects) {
		hit = glm::vec3(0.0f);
		normal = glm::vec3(0.0f);
	} else {
		tnear += 0.00001f;

		// march ray through surface
		float step = (tfar - tnear) / steps;

		// set sign to unknown at the beginning
		char sign = 0;
		
		// find location where sign is defined first
		int i = 0;
		for (i = 1; i < steps-1; i++) { // -1 to not get too close to tfar
			glm::vec3 target = org + dir * (tnear + i * step);
			float implicitValue; glm::vec3 implicitNormal;
			
			evaluateGrid(target, implicitValue, implicitNormal);
			
			if (implicitValue < 0.0f) {
				sign = -1;
				break;
			} else if (implicitValue > 0.0f) {
				sign = 1;
				break;
			}
		}
		
		// trace until sign changes
		if (sign) {
			for(; i < steps-1; i++) {
				glm::vec3 target = org + dir * (tnear + i * step);
				float implicitValue; glm::vec3 implicitNormal;
				
				evaluateGrid(target, implicitValue, implicitNormal);
				
				if ((sign == 1 && implicitValue < 0.0f) || (sign == -1 && implicitValue > 0.0f)) { // ray entered surface
					float tStart = tnear + (i-1) * step;
					step = (tnear + i * step - tStart) / steps;
					
					for (int j = steps-1; j >= 0; j--) {
						target = org + dir * (tStart + j * step);
						
						evaluateGrid(target, implicitValue, normal);
						
						if ((sign == 1 && implicitValue > 0.0f) || (sign == -1 && implicitValue < 0.0f)) {
							hit = target;
							normal = implicitNormal;
							break;
						}
					}
					break;
				}
			}
		}
	}
	
	d_output[idx].first = hit;
 	d_output[idx].second = normal;
}

struct IsNotZero {
	__host__ __device__
	bool operator()(const PointNormalPair& a) {
		return !(a.first.x == 0.0f && a.first.y == 0.0f && a.first.z == 0.0f);
	}
};

void launchRaymarchKernel(const util::Grid3D& volume, const util::RayVector& rays, unsigned steps, PointNormalData& results) {
	// configure device
	check( cudaDeviceSetCacheConfig( cudaFuncCachePreferL1 ), "cudaDeviceSetCacheConfig" );
	
	// copy bbox data to const
	check( cudaMemcpyToSymbol(c_bboxMin, glm::value_ptr(volume.bounds().min()), sizeof(float3)), "cudaMemcpyToSymbol" );
	check( cudaMemcpyToSymbol(c_bboxMax, glm::value_ptr(volume.bounds().max()), sizeof(float3)), "cudaMemcpyToSymbol" );
	
	// initialize grid textures
	cudaArray* d_volumeArray;
	Point3i dims = volume.dimensions();
	const cudaExtent volumeSize = make_cudaExtent(dims.x, dims.y, dims.z);
	
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float4>();
	check( cudaMalloc3DArray(&d_volumeArray, &channelDesc, volumeSize), "cudaMalloc3DArray" );
	
	cudaMemcpy3DParms copyParams = {0};
	copyParams.srcPtr   = make_cudaPitchedPtr((void*)volume.data(), volumeSize.width*sizeof(float)*4, volumeSize.width, volumeSize.height);
	copyParams.dstArray = d_volumeArray;
	copyParams.extent   = volumeSize;
	copyParams.kind     = cudaMemcpyHostToDevice;
	check( cudaMemcpy3D(&copyParams), "cudaMemcpy3D" );

	tex_voxels.normalized = true;
	tex_voxels.filterMode = cudaFilterModeLinear;
	tex_voxels.addressMode[0] = cudaAddressModeClamp;
	tex_voxels.addressMode[1] = cudaAddressModeClamp;
	tex_voxels.addressMode[2] = cudaAddressModeClamp;

	check( cudaBindTextureToArray(tex_voxels, d_volumeArray, channelDesc), "cudaBindTextureToArray" );
	
	// initialize ray array
 	glm::vec3* d_rays;
	check( cudaMalloc(&d_rays, sizeof(glm::vec3) * rays.size()), "cudaMalloc" );
	check( cudaMemcpy2D(d_rays, sizeof(glm::vec3), &rays[0].d, 2 * sizeof(glm::vec3), sizeof(glm::vec3), rays.size(), cudaMemcpyHostToDevice), "cudaMemcpy2D" );
	
	// initialize output arrays
	PointNormalPair* d_output;
	check( cudaMalloc(&d_output, sizeof(PointNormalPair) * rays.size()), "cudaMalloc" );

	// launch kernel
	dim3 block(256, 1, 1);
	dim3 grid((rays.size() + block.x - 1) / block.x, 1, 1);
	
	raymarchKernel<<<grid, block>>>(rays.front().o, d_rays, d_output, rays.size(), steps);
	check( cudaDeviceSynchronize(), "kernel launch" );
	
	// retrieve intersection data
	thrust::device_ptr<PointNormalPair> out = thrust::device_pointer_cast(d_output);
	thrust::device_vector<PointNormalPair> d_results;
	d_results.resize(rays.size());
	
	thrust::device_vector<PointNormalPair>::iterator end;
	checkThrust( end = thrust::copy_if(out, out + rays.size(), d_results.begin(), IsNotZero()) , "copy_if" );
	results.resize(end - d_results.begin());
	checkThrust( thrust::copy(d_results.begin(), end, results.begin()), "copy" );
	
	// cleanup
	d_results.resize(0); d_results.shrink_to_fit();
	check( cudaFree(d_rays), "cudaFree");
	check( cudaFree(d_output), "cudaFree");
	check( cudaFreeArray(d_volumeArray), "cudaFreeArray" );
}
