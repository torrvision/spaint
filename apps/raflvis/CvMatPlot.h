/**
 * tvgplot: CvMatPlot.h
 */

#ifndef H_TVGPLOT_CVMATPLOT
#define H_TVGPLOT_CVMATPLOT

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * \brief An instance of this class allows 2D plotting in a OpenCV matrix.
 */
class CvMatPlot
{
  //#################### PRIVATE TYPEDEFS #################### 
private:
  typedef cv::Scalar Colour;

  //#################### PRIVATE MEMBER VARIABLES #################### 
private:
  /** The absolute length of the visible axis in both the x and y directions. */
  float m_axesLength;

  /** The image on which to colour pixels or draw shapes. */
  cv::Mat m_canvas;

  /** The origin of the cartesian coordinate system. */
  cv::Point2f m_cartesianOriginInImage;

  /** The figure number used to identify multiple windows which share the same name. */
  size_t m_figureNumber;

  /** The height of the underlying image axes in pixels. */
  size_t m_imageHeight;

  /** The width of the underlying image axes in pixels. */
  size_t m_imageWidth; 

  /** The count used to identify images when a stream is saved. */
  int m_saveCounter;

  /** The scale along the image height used to convert from cartesian coordinates to image coordinates. */
  float m_scaleHeight;

  /** The scale along the image width used to convert from cartesian coordinates to image coordinates. */
  float m_scaleWidth;

  /** The name to display on the window. */
  std::string m_windowName;

  //#################### CONSTRUCTOR #################### 
public:
  /**
   * \brief Constructs an OpenCV image for drawing. 
   *
   * \param figureNumber    The number used to identify multiple windows which share the same name.
   * \param figureName      The name to display on the window.
   * \param imageWidth      The width of the image axes in pixels.
   * \param imageHeight     The height of the image axes in pixels.
   * \param axesLength      The absolute length of the visible axis in both the x and y directions.
   */
  CvMatPlot(size_t figureNumber, std::string figureName = "Drawing pad", size_t imageWidth = 700, size_t imageHeight = 700, int axesLength = 5);

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Draws the cartesian axes.
   *
   * \param colour  The colour of the axes.
   */
  void cartesian_axes(const Colour& colour);

  /**
   * \brief Draws a point in cartesian coordinates.
   *
   * \param point     The image point.
   * \param colour    The colour of hte point.
   * \param radius    The radius of the point.
   * \param thickness The thickness of the point.
   */
  void cartesian_point(cv::Point2f point, const Colour& colour, int radius = 2, int thickness = -1);
  
  /**
   * \brief Sets all the pixel values in the image to black.
   */
  void clf();

  /**
   * \brief Gets the current height of the image.
   *
   * \return The height of the cv::Mat image being used for drawing.
   */
  size_t height() const;
  
  /**
   * \brief Draws a line in image coordinates.
   *
   * \param point1  The first extremity of the line.
   * \param point2  The second extremity of the line.
   * \param colour  The colour of the line.
   * \param thickness The thickness of the line.
   */
  void image_line(cv::Point2f point1, cv::Point2f point2, const Colour& colour, int thickness = 1);

  /**
   * \brief Draws a point in image coordinates.
   * 
   * \param point      The image point.
   * \param colour     The colour of the point.
   * \param radius     The The radius of the point.
   * \param thickness  The thickness of the point. 
   */
  void image_point(const cv::Point2f& point, const Colour& colour, int radius = 5, int thickness = -1);

  /**
   * \brief Draws text in image coordinates.
   *
   * \param text       The test to be drawn.
   * \param poitition  The x-y position at which to draw the text.
   * \param colour     The colour of the text.
   * \param scale      The size of the text.
   * \param thickness  The thickness of the text font.
   */
  void image_text(std::string text, cv::Point position, const Colour& colour, double scale = 1.0, int thickness=2);

  /**
   * \brief Draws a line graph.
   *
   * \param values   The values making up the line graph.
   * \param colour   The colour of the line graph.
   */
  void line_graph(const std::vector<float>& values, const Colour& colour);

  /**
   * \brief Saves the current image to file.
   */
  void save(const std::string& path = "");

  /**
   * \brief Displays the current image in a window.
   */
  void show();

  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  /**
   * \brief Converts the cartesian coordinate points to image coordinates.
   *
   * \param axesPoint   The point in cartesian coordinates.
   * \return            The point in image coordinates.
   */
  cv::Point2f axes2image(const cv::Point2f axesPoint);

  /**
   * \brief Converts a colour in RGB format to a colour in BGR format compatible with OpenCV.
   */
  cv::Scalar rgb2bgr(const cv::Scalar& colour);
};

#endif

