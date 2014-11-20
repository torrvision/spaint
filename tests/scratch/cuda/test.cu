#include "test.h"

#include "multiply.cuh"

__global__ void simple(float *x)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  x[tid] = multiply(x[tid]);
}

void execute_simple_kernel(float *x, int numBlocks, int threadsPerBlock)
{
  simple<<<numBlocks,threadsPerBlock>>>(x);
}

__global__ void copy_1d_texture(cudaTextureObject_t input, unsigned char *output)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  output[tid] = tex1Dfetch<unsigned char>(input, tid);
}

void execute_copy_1d_texture_kernel(cudaTextureObject_t input, unsigned char *output, int numBlocks, int threadsPerBlock)
{
  copy_1d_texture<<<numBlocks,threadsPerBlock>>>(input, output);
}

__global__ void copy_2d_texture(cudaTextureObject_t input, unsigned char *output, int width, int height)
{
  int x = threadIdx.x + blockDim.x * blockIdx.x;
  int y = threadIdx.y + blockDim.y * blockIdx.y;
  if(x < width && y < height)
  {
    output[y * width + x] = tex2D<unsigned char>(input, x, y);
  }
}

void execute_copy_2d_texture_kernel(cudaTextureObject_t input, unsigned char *output, int width, int height, int blockWidth, int blockHeight)
{
  dim3 dimBlock(blockWidth, blockHeight);
  dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);
  copy_2d_texture<<<dimGrid,dimBlock>>>(input, output, width, height);
}
