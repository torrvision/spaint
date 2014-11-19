#ifndef H_TEST
#define H_TEST

//#################### FUNCTION DECLARATIONS ####################

extern "C" void execute_copy_1d_texture_kernel(cudaTextureObject_t input, unsigned char *output, int numBlocks, int threadsPerBlock);
extern "C" void execute_copy_2d_texture_kernel(cudaTextureObject_t input, unsigned char *output, int width, int height, int blockWidth, int blockHeight);
extern "C" void execute_simple_kernel(float *x, int numBlocks, int threadsPerBlock);

#endif
