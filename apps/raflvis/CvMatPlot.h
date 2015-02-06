/**
 * tvgplot: OcvPlot.h
 */

#ifndef H_TVGPLOT_OCVPLOT
#define H_TVGPLOT_OCVPLOT

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

  /** The figure number used to identify multiple windows which share the same name. */
  size_t m_figureNumber;

  /** The height of the underlying image axes in pixels. */
  size_t m_imageHeight;

  /** The width of the underlying image axes in pixels. */
  size_t m_imageWidth; 

  /** The origin of the cartesian coordinate system. */
  cv::Point2f m_origin;

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
   */
  CvMatPlot(size_t figureNumber, std::string figureName = "Drawing pad", size_t imageWidth = 700, size_t imageHeight = 700);

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
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
   * \brief Draws a point in cartesian coordinates.
   *
   * \param point     The image point.
   * \param colour    The colour of hte point.
   * \param radius    The radius of the point.
   * \param thickness The thickness of the point.
   */
  void cartesian_point(cv::Point2f point, const Colour& colour, int radius = 2, int thickness = -1);
  
  /**
   * \brief Draws text in image coordinates.
   *
   * \param text       The test to be drawn.
   * \param poitition  The x-y position at which to draw the text.
   * \param colour     The colour of the text.
   * \param scale      The size of the text.
   * \param thickness  The thickness of the text font.
   */
  void draw_text(std::string text, cv::Point position, const Colour& colour, double scale = 1.0, int thickness=2);

  /**
   * \brief Draws the cartesian axes.
   *
   * \param colour  The colour of the axes.
   */
  void cartesian_axes(const Colour& colour);

  void cartesian_unit_circle(const Colour& colour);
  void draw_line(cv::Point2f point1, cv::Point2f point2, const Colour& colour, int thickness=1);
  void draw_hyperplane(cv::Point2f normal, float bias, const Colour& colour);
  void draw_bars(const std::vector<float>& bars, const Colour& colour);
  void clf();
  void Save();
  void show();

  cv::Scalar rgb2bgr(const cv::Scalar& colour);

  size_t height() const;
  
  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  cv::Point2f axes2image(const cv::Point2f axesPoint);
};

#endif

