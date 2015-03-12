#include <stdexcept>
#include <stdio.h>
#include <arrayfire.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

enum Order
{
  ROW_MAJOR,
  COL_MAJOR
};

void ocvfig(const std::string& windowName, unsigned char *pixels, int width, int height, Order order) 
{
  if(order == Order::COL_MAJOR)
  {
    int step = height * sizeof(unsigned char);
    cv::Mat img(width, height, CV_8UC1, pixels, step);
    cv::transpose(img, img);
    cv::imshow(windowName, img);
  }
  else
  {
    int step = width * sizeof(unsigned char);
    cv::Mat img(height, width, CV_8UC1, pixels, step);
    cv::imshow(windowName, img);
  }
}

// 5x5 sigma-3 gaussian blur weights
static const float h_gauss[] = {
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0397,  0.0469,  0.0495,  0.0469,  0.0397,
    0.0375,  0.0443,  0.0469,  0.0443,  0.0375,
    0.0318,  0.0375,  0.0397,  0.0375,  0.0318,
};

void img_test_demo(const af::array& source)
{
  static int rows = source.dims(0);
  static int cols = source.dims(1);

  /*static int width = source.dims(1);
  static int height = source.dims(0);*/
  static int width = cols;
  static int height = rows;
  std::cout << "width = " << width << ", height = " << height << '\n';
  ocvfig("source", source.as(u8).host<unsigned char>(), width, height, Order::COL_MAJOR);

  // load convolution kernels
  af::array gauss_k = af::array(5, 5, h_gauss);

  // Convolve with gaussian kernel.
  af::array transformedArray = af::convolve(source, gauss_k);

  // Threshold the image.
  af::array thresholdedArray = transformedArray < 50.0f;

  // Connected component analysis.
  af::array connectedComponents = af::regions(thresholdedArray);

  int maximum = af::max<int>(connectedComponents);
  for(int i = 1; i <= 2; ++i)
  {
    af::array tmp = (connectedComponents == i) * 255.0f;
    ocvfig("tmp " + std::to_string(i), tmp.as(u8).host<unsigned char>(), width, height, Order::COL_MAJOR);
  }

  thresholdedArray = 255.0f * thresholdedArray;
  ocvfig("thresholded", thresholdedArray.as(u8).host<unsigned char>(), width, height, Order::COL_MAJOR);
}

int main(int argc, char **argv)
{
  // Deal with the input.
  if(argc != 2)
  {
    throw std::runtime_error("You need to enter the input path to a grayscale image, and an output path to save the transformed image.\n");
  }
  std::string inPath(argv[1]);

  int device = 0;
  std::cout << "device=" << device << std::endl;
  std::cout << "image input path =" << inPath << std::endl;
  
  try
  {
    af::deviceset(device);
    af::info();
    std::cout << "Testing the arrayfire image processing.\n";

    // Load image
    af::array img_gray = af::loadimage(inPath.c_str(), false); // 1 channel grayscale [0-255]
    cv::Mat cv_img_gray = cv::imread(inPath, CV_LOAD_IMAGE_GRAYSCALE);
    ocvfig("opencv image", (unsigned char*)cv_img_gray.data, cv_img_gray.cols, cv_img_gray.rows, Order::ROW_MAJOR);

    for(;;)
    {
      img_test_demo(img_gray);

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
