//###
#if 1

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <sl/Camera.hpp>

//#################### TYPEDEFS ####################

namespace sl
{

typedef boost::shared_ptr<Camera> Camera_Ptr;
typedef boost::shared_ptr<const Camera> Camera_CPtr;

}

//#################### FUNCTIONS ####################

void destroy_zed(sl::Camera *zed)
{
  zed->close();
  delete zed;
}

sl::Camera_Ptr init_zed()
{
  sl::Camera *zed = new sl::Camera;

  sl::InitParameters params;
  params.camera_resolution = sl::RESOLUTION_VGA;
  params.depth_mode = sl::DEPTH_MODE_QUALITY;
  params.coordinate_units = sl::UNIT_METER;

  sl::ERROR_CODE err = zed->open(params);
  if(err == sl::SUCCESS)
  {
    return sl::Camera_Ptr(zed, destroy_zed);
  }
  else
  {
    delete zed;
    return sl::Camera_Ptr();
  }
}

/**
 * Conversion function between sl::Mat and cv::Mat
 */
cv::Mat slMat2cvMat(sl::Mat& input)
{
  // Mapping between MAT_TYPE and CV_TYPE
  int cv_type = -1;
  switch(input.getDataType())
  {
    case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
    default: break;
  }

  // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
  // cv::Mat and sl::Mat will share a single memory structure
  return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

int main()
{
  sl::Camera_Ptr zed = init_zed();
  if(!zed)
  {
    std::cerr << "Error: Could not initialise Zed camera\n";
    return EXIT_FAILURE;
  }

  sl::Resolution imgSize = zed->getResolution();
  sl::Mat colour(imgSize.width, imgSize.height, sl::MAT_TYPE_8U_C4);
  cv::Mat cvColour = slMat2cvMat(colour);

  sl::RuntimeParameters params;
  params.sensing_mode = sl::SENSING_MODE_STANDARD;

  sl::Mat depth;

  if(zed->grab(params) == sl::SUCCESS)
  {
    zed->retrieveMeasure(depth);
    zed->retrieveImage(colour, sl::VIEW_LEFT);

    cv::imshow("Colour", cvColour);

    cv::waitKey();
  }

  return 0;
}

#endif

//###
#if 0

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV.                              **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
void printHelp();

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", errorCode2str(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
            zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);

            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image", image_ocv);
            cv::imshow("Depth", depth_image_ocv);

            // Handle key event
            key = cv::waitKey(10);
        }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}

#endif

//###
#if 0

#include <sl/Camera.hpp>
using namespace sl;

int main(int argc, char **argv)
{
  // Create a ZED camera object
  Camera zed;

  // Set configuration parameters
  InitParameters init_params;
  init_params.sdk_verbose = false; // Disable verbose mode

  // Open the camera
  ERROR_CODE err = zed.open(init_params);
  if (err != SUCCESS)
      exit(-1);

  // Get camera information (ZED serial number)
  int zed_serial = zed.getCameraInformation().serial_number;
  printf("Hello! This is my serial number: %d\n", zed_serial);

  // Close the camera
  zed.close();
  return 0;
}

#endif
