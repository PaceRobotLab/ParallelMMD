
#include "cv2sim\StdAfx.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <cv.h>
#define TX 32
#define TY 32

#include <stdio.h>

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

__global__ void addKernel(int *c, const int *a, const int *b)
{
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}
#if 0
_global_ void kernel(uchar4 *d_out, int w, int h, int2 pos)
{
	const int c = blockIdx.x*blockDim.x + threadIdx.x;
	const int r = blockIdx.y*blockDim.y + threadIdx.y;
	if ((c >= w) || (r >= h)) return;
	const int i = c + r*w;
	const int
}
#endif

void cv2simSmall_grace_cuda(int matches[][4], int nMatches, int height, int width, int step, int channels, double* mWeights,
	char* img1ptr, char* img2ptr, char* maskptr, double eSum, double mSum){

	int offset = 0, newmask, newval;
	double sx = 5.0, sy = 5.0, eGain = 50;
	int j = 0, i = 0, k = 0;
	int id = 0, jd = 0, w = 0;
	for (j = 0; j < height; j++){
		for (i = 0; i < width; i++){
			for (k = 0; k < channels; k++) {
				eSum = 0.0;
				for (w = 0; w < nMatches; w++) {
					id = (i - matches[w][2]); jd = (j - matches[w][3]);
					eSum += (mWeights[w] / mSum)*exp((double)-(id*id / (2.0*sx*sx) + jd*jd / (2.0*sy*sy)));
				}
				eSum *= eGain;

				offset = j*step + i*channels + k;
				newmask = (int)(eSum * 255.0);
				if (newmask > 255) newmask = 255; else if (newmask < 0) newmask = 0;

				if (newmask > 0)
					newval = (int)((double)img1ptr[offset] / (double)maskptr[offset]);
				else newval = img1ptr[offset];

				if (newval > 255) newval = 255; else if (newval < 0) newval = 0;

				img2ptr[offset] = newval; //min( 255, max(1,newval));
				maskptr[offset] = newmask;
				//if (i%2==0) 	fprintf(mlog,"%d,",(unsigned char)maskptr[offset]);
				//if (i==width-1) fprintf(mlog,"0\n");
			}
		}
	}
}

__global__ void kernel_grace(int* matches, int nMatches, int height, int width, int step, int channels, double* mWeights,
	char* img1ptr, char* img2ptr, char* maskptr, double eSum, double mSum){
	

	int offset = 0, newmask, newval;
	double sx = 5.0, sy = 5.0, eGain = 50;
	int j = blockIdx.x, i = threadIdx.x, k = 0;
	int id = 0, jd = 0, w = 0;
	//j = 35, i = 15, k = 0
	//w = 300, h = 500
	//j * w + i = 35 * 300 + 15 = 10515
	eSum = 0.0;
	for (w = 0; w < nMatches; w++) {
		id = (i - matches[w * 4 + 2]); jd = (j - matches[w * 4 + 3]);
		eSum += (mWeights[w] / mSum)*exp((float)-(id*id / (2.0*sx*sx) + jd*jd / (2.0*sy*sy)));
	}
	eSum *= eGain;

	offset = j*step + i*channels + k;
	newmask = (int)(eSum * 255.0);
	if (newmask > 255) newmask = 255; else if (newmask < 0) newmask = 0;

	if (newmask > 0)
		newval = 255-(int)((double)img1ptr[offset] / (double)maskptr[offset]);
	else newval = img1ptr[offset];

	if (newval > 255) newval = 255; else if (newval < 0) newval = 0;

	img2ptr[offset] = newval; //min( 255, max(1,newval));
	maskptr[offset] = newmask;

#if 0
	int ind = threadIdx.x;
	int offset = 0, newmask, newval;
	double sx = 5.0, sy = 5.0, eGain = 50;
	int j = ind, i = 0, k = 0;
	int id = 0, jd = 0, w = 0;
	for (int i = 0; i < width; ++i){
		//j = 35, i = 15, k = 0
		//w = 300, h = 500
		//j * w + i = 35 * 300 + 15 = 10515
		eSum = 0.0;
		for (w = 0; w < nMatches; w++) {
			id = (i - matches[w * 4 + 2]); jd = (j - matches[w * 4 + 3]);
			eSum += (mWeights[w] / mSum)*exp((double)-(id*id / (2.0*sx*sx) + jd*jd / (2.0*sy*sy)));
		}
		eSum *= eGain;

		offset = j*step + i*channels + k;
		newmask = (int)(eSum * 255.0);
		if (newmask > 255) newmask = 255; else if (newmask < 0) newmask = 0;

		if (newmask > 0)
			newval = (int)((double)img1ptr[offset] / (double)maskptr[offset]);
		else newval = img1ptr[offset];

		if (newval > 255) newval = 255; else if (newval < 0) newval = 0;

		img2ptr[offset] = newval; //min( 255, max(1,newval));
		maskptr[offset] = newmask;
	}
#endif
}

void cv2simSmall_grace_cuda_2(int matches[][4], int nMatches, int height, int width, int step, int channels, double* mWeights,
	char* img1ptr, char* img2ptr, char* maskptr, double eSum, double mSum){

	cudaError_t cudaStatus;
	char *dev_img1ptr = 0;
	char *dev_img2ptr = 0;
	char *dev_maskptr = 0;
	int *dev_matches = 0;
	double* dev_mWeights = 0;
	GraceTimer timer;

	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	cudaStatus = cudaMalloc((void**)&dev_img1ptr, (height*step) * sizeof(char));
	cudaStatus = cudaMalloc((void**)&dev_img2ptr, (height*step) * sizeof(char));
	cudaStatus = cudaMalloc((void**)&dev_maskptr, (height*step) * sizeof(char));
	cudaStatus = cudaMalloc((void**)&dev_matches, nMatches * 4 * sizeof(int));
	cudaStatus = cudaMalloc((void**)&dev_mWeights, 1000 * sizeof(double));
	//timer.step("malloc fee: %.2f ms\n");

	cudaStatus = cudaMemcpy(dev_img1ptr, img1ptr, (height*step) * sizeof(char), cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(dev_img2ptr, img2ptr, (height*step) * sizeof(char), cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(dev_maskptr, maskptr, (height*step) * sizeof(char), cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(dev_matches, matches, nMatches * 4 * sizeof(int), cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(dev_mWeights, mWeights, 1000 * sizeof(double), cudaMemcpyHostToDevice);
	//timer.step("memcpy fee: %.2f ms\n");

	//size_t size = height*width*channels;
	kernel_grace << <height, width >> >(dev_matches, nMatches, height, width, step, channels, dev_mWeights,
		dev_img1ptr, dev_img2ptr, dev_maskptr, eSum, mSum);

	cudaStatus = cudaGetLastError();
	cudaStatus = cudaDeviceSynchronize();
	//timer.step("call kernel fee: %.2f ms\n");
	cudaStatus = cudaMemcpy(maskptr, dev_maskptr, (height*step) * sizeof(char), cudaMemcpyDeviceToHost);
	cudaStatus = cudaMemcpy(img2ptr, dev_img2ptr, (height*step) * sizeof(char), cudaMemcpyDeviceToHost);
	cudaFree(dev_img1ptr);
	cudaFree(dev_img2ptr);
	cudaFree(dev_maskptr);
	cudaFree(dev_matches);
	cudaFree(dev_mWeights);
	//cudaStatus = cudaDeviceReset();
	//timer.step("result fee %.2f ms\n");
}

int calcByCU(){
	const int arraySize = 5;
	const int a[arraySize] = { 1, 2, 3, 4, 5 };
	const int b[arraySize] = { 10, 20, 30, 40, 50 };
	int c[arraySize] = { 0 };

	// Add vectors in parallel.
	cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addWithCuda failed!");
		return 1;
	}

	printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
		c[0], c[1], c[2], c[3], c[4]);

	// cudaDeviceReset must be called before exiting in order for profiling and
	// tracing tools such as Nsight and Visual Profiler to show complete traces.
	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceReset failed!");
		return 1;
	}
	return 1 + 3;
}

int main11()
{
    const int arraySize = 5;
    const int a[arraySize] = { 1, 2, 3, 4, 5 };
    const int b[arraySize] = { 10, 20, 30, 40, 50 };
    int c[arraySize] = { 0 };

    // Add vectors in parallel.
    cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addWithCuda failed!");
        return 1;
    }

    printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
        c[0], c[1], c[2], c[3], c[4]);

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        return 1;
    }

    return 0;
}

// Helper function for using CUDA to add vectors in parallel.
cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size)
{
    int *dev_a = 0;
    int *dev_b = 0;
    int *dev_c = 0;
    cudaError_t cudaStatus;

    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        goto Error;
    }

    // Allocate GPU buffers for three vectors (two input, one output)    .
    cudaStatus = cudaMalloc((void**)&dev_c, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    cudaStatus = cudaMalloc((void**)&dev_b, size * sizeof(int));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    // Copy input vectors from host memory to GPU buffers.
    cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    cudaStatus = cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    // Launch a kernel on the GPU with one thread for each element.
    addKernel<<<1, size>>>(dev_c, dev_a, dev_b);

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        goto Error;
    }
    
    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
        goto Error;
    }

    // Copy output vector from GPU buffer to host memory.
    cudaStatus = cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

Error:
    cudaFree(dev_c);
    cudaFree(dev_a);
    cudaFree(dev_b);
    
    return cudaStatus;
}
