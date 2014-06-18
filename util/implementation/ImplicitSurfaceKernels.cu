/*
 * CUDA source file for ImplicitSurface class.
 */

#define CUDA_VERSION 4020

#include <util/interface/ImplicitSurface.hpp>

__global__
void raymarchKernel() {
	
}

void launchRaymarchKernel() {
	dim3 grid(1,1,1), block(1,1,1);
	raymarchKernel<<<grid, block>>>();
}
