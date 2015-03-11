#include <stdexcept>
#include <stdio.h>
#include <arrayfire.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void ocvfig(unsigned char *pixels, int width, int height) 
{
  cv::Mat img(height, width, CV_8UC1, pixels);
  cv::imshow("test", img);
}

// 5x5 sigma-3 gaussian blur weights
static const float h_gauss[] = {
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0397,  0.0469,  0.0495,  0.0469,  0.0397,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
};

void img_test_demo(const std::string& inPath, const std::string& outPath)
{
  //load image
  af::array img_gray = af::loadimage(inPath.c_str(), false); // 1 channel grayscale [0-255]

  // load convolution kernels
  af::array gauss_k = af::array(5, 5, h_gauss);

  // Convolve with gaussian kernel.
  af::array transformedArray = af::convolve(img_gray, gauss_k);

  // Threshold the image.
  af::array thresholdedArray = transformedArray < 50.0f;

  // Connected component analysis.
  af::array connectedComponents = af::regions(thresholdedArray);

  int maximum = af::max<int>(connectedComponents);
  for(int i = 1; i <= maximum; ++i)
  {
    char buffer[200];
    sprintf(buffer, "connected-component-%d.jpg", i);
    af::array tmp = (connectedComponents == i);
    af::saveImage(buffer, tmp);
  }

  unsigned char *pixels = thresholdedArray.as(u8).host<unsigned char>();
  ocvfig(pixels, thresholdedArray.dims(0), thresholdedArray.dims(1));
  // Save the image to file.
  af::saveimage(outPath.c_str(), thresholdedArray);
}

int main(int argc, char **argv)
{
  // Deal with the input.
  if(argc != 3)
  {
    throw std::runtime_error("You need to enter the input path to a grayscale image, and an output path to save the transformed image.\n");
  }
  std::string inPath(argv[1]);
  std::string outPath(argv[2]);

  int device = 0;
  std::cout << "device=" << device << std::endl;
  std::cout << "image input path =" << inPath << std::endl;
  std::cout << "image output path =" << outPath << std::endl;
  
  try
  {
    af::deviceset(device);
    af::info();
    std::cout << "Testing the arrayfire image processing.\n";
    for(;;)
    {
      img_test_demo(inPath, outPath);

      if(cv::waitKey(30) == 'q')
        break;
    }

  }
  catch (af::exception& ae)
  {
    std::cout << ae.what() << std::endl;
    throw;
  }

  return 0;
}
