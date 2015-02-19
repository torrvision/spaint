/**
 * tvgplot: CvPlotter.h
 */

#ifndef H_TVGPLOT_CVPLOTTER
#define H_TVGPLOT_CVPLOTTER

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * \brief An instance of this class allows basic drawing in an OpenCV image,
 * as well as plotting basic shapes in Cartesian coordinates.
 */
class CvPlotter
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The absolute length of the visible axis in both the x and y directions. */
  float m_axesLength;

  /** The image on which to colour pixels or draw shapes. */
  mutable cv::Mat m_canvas;

  /** The origin of the Cartesian coordinate system. */
  cv::Point2f m_cartesianOriginInImage;

  /** The figure number used to identify multiple windows which share the same name. */
  size_t m_figureNumber;

  /** The height of the underlying image axes in pixels. */
  size_t m_imageHeight;

  /** The width of the underlying image axes in pixels. */
  size_t m_imageWidth;

  /** The count used to identify images when a stream is saved. */
  int m_saveCounter;

  /** The scale along the image height used to convert from Cartesian coordinates to image coordinates. */
  float m_scaleHeight;

  /** The scale along the image width used to convert from Cartesian coordinates to image coordinates. */
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
  CvPlotter(size_t figureNumber, std::string figureName = "Drawing pad", size_t imageWidth = 700, size_t imageHeight = 700, int axesLength = 5);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Draws the Cartesian axes.
   *
   * \param colour  The colour of the axes.
   */
  void cartesian_axes(const cv::Scalar& colour) const;

  /**
   * \brief Draws a point in Cartesian coordinates.
   *
   * \param point     The image point.
   * \param colour    The colour of the point.
   * \param radius    The radius of the point.
   * \param thickness The thickness of the point.
   */
  void cartesian_point(cv::Point2f point, const cv::Scalar& colour, int radius = 2, int thickness = -1) const;

  /**
   * \brief Sets all the pixel values in the image to black.
   */
  void clf() const;

  /**
   * \brief Gets the current height of the image.
   *
   * \return The height of the cv::Mat image being used for drawing.
   */
  size_t height() const;

  /**
   * \brief Draws a line in image coordinates.
   *
   * \param point1    The first extremity of the line.
   * \param point2    The second extremity of the line.
   * \param colour    The colour of the line.
   * \param thickness The thickness of the line.
   */
  void image_line(cv::Point2f point1, cv::Point2f point2, const cv::Scalar& colour, int thickness = 1)const;

  /**
   * \brief Draws a point in image coordinates.
   *
   * \param point      The image point.
   * \param colour     The colour of the point.
   * \param radius     The The radius of the point.
   * \param thickness  The thickness of the point.
   */
  void image_point(const cv::Point2f& point, const cv::Scalar& colour, int radius = 5, int thickness = -1) const;

  /**
   * \brief Draws text in image coordinates.
   *
   * \param text       The test to be drawn.
   * \param poitition  The x-y position at which to draw the text.
   * \param colour     The colour of the text.
   * \param scale      The size of the text.
   * \param thickness  The thickness of the text font.
   */
  void image_text(std::string text, cv::Point position, const cv::Scalar& colour, double scale = 1.0, int thickness=2) const;

  /**
   * \brief Draws a line graph.
   *
   * \param values   The values making up the line graph.
   * \param colour   The colour of the line graph.
   */
  void line_graph(const std::vector<float>& values, const cv::Scalar& colour) const;

  /**
   * \brief Saves the current image to file.
   */
  void save(const std::string& path = "");

  /**
   * \brief Displays the current image in a window.
   */
  void show() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Converts the Cartesian coordinate points to image coordinates.
   *
   * \param axesPoint   The point in Cartesian coordinates.
   * \return            The point in image coordinates.
   */
  cv::Point2f axes2image(const cv::Point2f axesPoint) const;

  /**
   * \brief Converts a colour in RGB format to a colour in BGR format compatible with OpenCV.
   *
   * \return The colour in BGR format.
   */
  cv::Scalar rgb2bgr(const cv::Scalar& colour) const;
};

#endif

