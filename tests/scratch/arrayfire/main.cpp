#include <stdexcept>
#include <stdio.h>
#include <arrayfire.h>

// 5x5 sigma-3 gaussian blur weights
static const float h_gauss[] = {
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0397,  0.0469,  0.0495,  0.0469,  0.0397,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
};

void img_test_demo(bool console, const std::string& pathToImage)
{
  // load convolution kernels
  af::array gauss_k = af::array(5, 5, h_gauss);

  //load image
  af::array img_gray = af::loadimage(pathToImage.c_str(), false); // 1 channel grayscale [0-255]
}

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    throw std::runtime_error("You need to enter the path to a grayscale image\n");
  }
  std::string pathToImage(argv[1]);

  int device = 0;
  bool console = false;

  std::cout << "device=" << device << ", console=" << console << std::endl;
  std::cout << "image path =" << pathToImage << std::endl;

  try
  {
    af::deviceset(device);
    af::info();
    std::cout << "Testing the arrayfire image processing.\n";
    img_test_demo(console, pathToImage);
  }
  catch (af::exception& ae)
  {
    std::cout << ae.what() << std::endl;
    throw;
  }

  if(!console)
  {
    std::cout << "Hit [Enter] ...";
    getchar();
  }

  return 0;
}
