#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

#include <cuda_runtime.h>
#include <helper_cuda.h>

#include "test.h"

void test_simple()
{
  // Set the size of the array, the number of threads per block and the number of blocks.
  int N = 64;
  int threadsPerBlock = 16;
  int numBlocks = N / threadsPerBlock;

  // Set up the array on the host.
  std::vector<float> h_arr(N);
  for(int i = 0; i < N; ++i)
  {
    h_arr[i] = static_cast<float>(i);
  }

  // Copy the array across to the device.
  float *d_arr;
  checkCudaErrors(cudaMalloc((void**)&d_arr, N * sizeof(float)));
  checkCudaErrors(cudaMemcpy(d_arr, &h_arr[0], N * sizeof(float), cudaMemcpyHostToDevice));

  // Execute the test kernel and retrieve the results.
  execute_simple_kernel(d_arr, numBlocks, threadsPerBlock);
  cudaError_t err = cudaGetLastError();
  if(err != cudaSuccess)
  {
    std::cout << "Error: " << cudaGetErrorString(err) << '\n';
    return;
  }
  checkCudaErrors(cudaMemcpy(&h_arr[0], d_arr, N * sizeof(float), cudaMemcpyDeviceToHost));

  // Output the results.
  std::copy(h_arr.begin(), h_arr.end(), std::ostream_iterator<float>(std::cout, "\n"));

  // Free the memory on the device.
  cudaFree(d_arr);
}

void test_copy_1d_texture()
{
  // Set the size of the texture, the number of threads per block and the number of blocks.
  int N = 32;
  int threadsPerBlock = 16;
  int numBlocks = N / threadsPerBlock;

  // Set up the texture buffer on the host.
  std::vector<unsigned char> h_buffer(N);
  for(int i = 0; i < N; ++i)
  {
    h_buffer[i] = static_cast<unsigned char>(i);
  }

  // Copy the texture buffer across to the device.
  unsigned char *d_buffer;
  checkCudaErrors(cudaMalloc((void**)&d_buffer, N));
  checkCudaErrors(cudaMemcpy(d_buffer, &h_buffer[0], N, cudaMemcpyHostToDevice));

  // Create the texture object.
  cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeLinear;
  resDesc.res.linear.devPtr = d_buffer;
  resDesc.res.linear.desc.f = cudaChannelFormatKindUnsigned;
  resDesc.res.linear.desc.x = 8; // bits per channel
  resDesc.res.linear.sizeInBytes = N;

  cudaTextureDesc texDesc;
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.readMode = cudaReadModeElementType;

  cudaTextureObject_t tex;
  cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);

  // Execute the kernel and retrieve the results.
  unsigned char *d_output;
  checkCudaErrors(cudaMalloc((void**)&d_output, N));
  execute_copy_1d_texture_kernel(tex, d_output, numBlocks, threadsPerBlock);
  cudaError_t err = cudaGetLastError();
  if(err != cudaSuccess)
  {
    std::cout << "Error: " << cudaGetErrorString(err) << '\n';
    return;
  }
  std::vector<unsigned char> h_output(N);
  checkCudaErrors(cudaMemcpy(&h_output[0], d_output, N, cudaMemcpyDeviceToHost));

  // Output the results.
  for(size_t i = 0; i < N; ++i)
  {
    std::cout << static_cast<int>(h_output[i]) << '\n';
  }

  // Clean up.
  cudaDestroyTextureObject(tex);
  cudaFree(d_buffer);
  cudaFree(d_output);
}

void test_copy_2d_texture()
{
  // Set the size of the texture.
  size_t width = 3;
  size_t height = 5;

  // Set up the texture buffer on the host.
  std::vector<unsigned char> h_buffer(width * height);
  size_t k = 0;
  for(size_t y = 0; y < height; ++y)
  {
    for(size_t x = 0; x < width; ++x)
    {
      h_buffer[y * width + x] = k++;
    }
  }

  // Copy the texture buffer across to the device.
  cudaArray *d_buffer;
  cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(8, 0, 0, 0, cudaChannelFormatKindUnsigned);
  checkCudaErrors(cudaMallocArray(&d_buffer, &channelDesc, width, height));
  checkCudaErrors(cudaMemcpy2DToArray(d_buffer, 0, 0, &h_buffer[0], width, width, height, cudaMemcpyHostToDevice));

  // Create the texture object.
  cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = d_buffer;

  cudaTextureDesc texDesc;
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.filterMode = cudaFilterModePoint;
  texDesc.readMode = cudaReadModeElementType;
  texDesc.normalizedCoords = 0;

  cudaTextureObject_t tex;
  cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);

  // Execute the kernel and retrieve the results.
  unsigned char *d_output;
  checkCudaErrors(cudaMalloc((void**)&d_output, width * height));
  execute_copy_2d_texture_kernel(tex, d_output, width, height, 2, 2);
  cudaError_t err = cudaGetLastError();
  if(err != cudaSuccess)
  {
    std::cout << "Error: " << cudaGetErrorString(err) << '\n';
    return;
  }
  std::vector<unsigned char> h_output(width * height);
  checkCudaErrors(cudaMemcpy(&h_output[0], d_output, width * height, cudaMemcpyDeviceToHost));
  
  // Output the results.
  for(size_t i = 0; i < width * height; ++i)
  {
    std::cout << static_cast<int>(h_output[i]) << '\n';
  }

  // Clean up.
  cudaDestroyTextureObject(tex);
  cudaFreeArray(d_buffer);
  cudaFree(d_output);
}

int main(int argc, char **argv)
{
  findCudaDevice(argc, const_cast<const char**>(argv));

  //test_simple();
  //test_copy_1d_texture();
  test_copy_2d_texture();

  return 0;
}
