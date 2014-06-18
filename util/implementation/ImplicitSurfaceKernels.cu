/*
 * CUDA source file for ImplicitSurface class.
 */

#define check(expr, msg) { \
	cudaError_t err; \
	err = expr; \
	if (err != cudaSuccess) { std::cerr << "CUDA ERROR (" << cudaGetErrorString(err) << "): " << msg << std::endl; } \
	}

#include <util/interface/Util.hpp>
#include <util/interface/ImplicitSurface.hpp>
#include <glm/interface/gtc/type_ptr.hpp>

#include <util/interface/cutil_math.h>

#include <limits.h>
#include <float.h>

// constant data
__constant__ float3 c_bboxMin;
__constant__ float3 c_bboxMax;
__constant__ int3 c_gridDims;

// textures
texture<float4, 3, cudaReadModeElementType> tex_voxels;

inline __device__ glm::vec3 make_vec3(float3 a) {
	return glm::vec3(a.x, a.y, a.z);
}

__device__
bool clipRayAgainstAABB(const glm::vec3& org, const glm::vec3& dir, float& tnear, float& tfar) {
	glm::vec3 T_1, T_2;
	double t_near = -FLT_MAX;
	double t_far = FLT_MAX;
	
	glm::vec3 min = make_vec3(c_bboxMin);
	glm::vec3 max = make_vec3(c_bboxMax);
	
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
			if (T_1[i] > t_near) {
				t_near = T_1[i];
			}
			if (T_2[i] < t_far) {
				t_far = T_2[i];
			}
			if ( (t_near > t_far) || (t_far < 0.0f) || (fabsf(t_far - t_near) < 0.0001f) ) {
				return false;
			}
		}
	}
	
	tnear = t_near; tfar = t_far;
	
	return true;
}

__device__
void evaluateGrid(const glm::vec3& target, float* value, glm::vec3* normal) {
	glm::vec3 coords = (target - make_vec3(c_bboxMin)) / (make_vec3(c_bboxMax) - make_vec3(c_bboxMin));
	float4 interp = tex3D(tex_voxels, coords.x, coords.y, coords.z);
	if (value) {
		*value = interp.w;
	}
	if (normal) {
		*normal = glm::vec3(interp.x, interp.y, interp.z);
	}
}

__global__
void raymarchKernel(glm::vec3* d_rays, glm::vec3* d_output, unsigned n, unsigned steps) {
	unsigned idx = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (idx > n) {
		return;
	}
	
	glm::vec3 org = d_rays[2*idx];
	glm::vec3 dir = d_rays[2*idx+1];
	
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
		bool sign, signWasSet = false;

		// trace forward through the bounding box
		for (unsigned i = 1; i < steps-1; i++) { // -1 to not get too close to tfar
			glm::vec3 target = org + dir * (tnear + i * step);
			float implicitValue; glm::vec3 implicitNormal;
			evaluateGrid(target, &implicitValue, &implicitNormal);
			
			if (!signWasSet) {
				if (implicitValue < 0.0f) {
					sign = false;
					signWasSet = true;
				} else if (implicitValue > 0.0f) {
					sign = true;
					signWasSet = true;
				}
			} else {
				bool enteredSurface = (sign == true && implicitValue < 0.0f) || (sign == false && implicitValue > 0.0f);
				if (enteredSurface) { // ray entered surface
				
					// compute bounds between this and the last step for backward trace
					float tStart = tnear + (i-1) * step;
					float tEnd = tnear + i * step;
					float smallStep = (tEnd - tStart) / steps;
					
					// trace backwards from step that changed the sign
					for (int j = steps-1; j >= 0; j--) {
						target = org + dir * (tStart + j * smallStep);
						evaluateGrid(target, &implicitValue, &normal);
						
						bool exitedSurface = (sign == true && implicitValue > 0.0f) || (sign == false && implicitValue < 0.0f);
						if (exitedSurface) {
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
	
	d_output[idx] = hit;
	d_output[n + idx] = normal;
}

void launchRaymarchKernel(const util::Grid3D& volume, const util::RayVector& rays, unsigned steps, PointNormalData& results) {
	// configure device
	check( cudaDeviceSetCacheConfig( cudaFuncCachePreferL1 ), "cudaDeviceSetCacheConfig" );
	
	// copy bbox data to const
	check( cudaMemcpyToSymbol(c_bboxMin, glm::value_ptr(volume.bounds().min()), sizeof(float3)), "cudaMemcpyToSymbol" );
	check( cudaMemcpyToSymbol(c_bboxMax, glm::value_ptr(volume.bounds().max()), sizeof(float3)), "cudaMemcpyToSymbol" );
	check( cudaMemcpyToSymbol(c_gridDims, glm::value_ptr(volume.dimensions()), sizeof(int3)), "cudaMemcpyToSymbol" );
	
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
	check( cudaMalloc(&d_rays, sizeof(glm::vec3) * 2 * rays.size()), "cudaMalloc" );
	check( cudaMemcpy(d_rays, rays.data(), sizeof(glm::vec3) * 2 * rays.size(), cudaMemcpyHostToDevice), "cudaMemcpy" );
	
	// initialize output array
	glm::vec3* d_output;
	check( cudaMalloc(&d_output, sizeof(glm::vec3) * 2 * rays.size()), "cudaMalloc" );
	
	// launch kernel
	dim3 block(256, 1, 1);
	dim3 grid((rays.size() + block.x - 1) / block.x, 1, 1);
	
	raymarchKernel<<<grid, block>>>(d_rays, d_output, rays.size(), steps);
	check( cudaDeviceSynchronize(), "kernel launch" );
	
	// retrieve intersection data
	glm::vec3* h_output = new glm::vec3[2 * rays.size()];
	check( cudaMemcpy(h_output, d_output, sizeof(glm::vec3) * 2 * rays.size(), cudaMemcpyDeviceToHost), "cudaMemcpy" );
	
	results.reserve(rays.size());
	for (unsigned i = 0; i < rays.size(); i++) {
		const glm::vec3& p = h_output[i];
		const glm::vec3& n = h_output[i + rays.size()];
		if (p != Point3f(0.0f)) {
			results.push_back( std::make_pair(p, n) );
		}
	}
	
	// cleanup
	delete[] h_output;
	check( cudaFree(d_rays), "cudaFree");
	check( cudaFree(d_output), "cudaFree");
}
