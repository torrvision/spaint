/**
 * raflvis: PlotWindow.h
 */

#ifndef H_RAFLVIS_PLOTWINDOW
#define H_RAFLVIS_PLOTWINDOW

#include <string>

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * \brief An instance of this class represents a window into which we can plot shapes and/or a graph.
 */
class PlotWindow
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The absolute length of the visible axis in both the x and y directions. */
  float m_axesLength;

  /** The image on which to draw the shapes or graph. */
  mutable cv::Mat m_canvas;

  /** The origin of the Cartesian coordinate system. */
  cv::Point2f m_cartesianOriginInCanvas;

  /** The height of the canvas in pixels. */
  size_t m_canvasHeight;

  /** The width of the canvas in pixels. */
  size_t m_canvasWidth;

  /** A counter recording how many times the canvas has been saved (this is used to name the files when saving a stream of image to disk). */
  int m_saveCounter;

  /** The vertical scaling factor by which to multiply when converting Cartesian coordinates to image coordinates. */
  float m_scaleHeight;

  /** The scale along the image width used to convert from Cartesian coordinates to image coordinates. */
  float m_scaleWidth;

  /** Constructs a window into which we can plot shapes and/or a graph. */
  std::string m_windowName;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an OpenCV image for drawing.
   *
   * \param windowName      The name to display on the window.
   * \param canvasWidth     The width of the canvas axes in pixels.
   * \param canvasHeight    The height of the canvas axes in pixels.
   * \param axesLength      The absolute length of the visible axes in both the x and y directions.
   */
  explicit PlotWindow(const std::string& windowName, size_t canvasWidth = 700, size_t canvasHeight = 700, int axesLength = 5);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Draws the Cartesian axes.
   *
   * \param colour  The colour of the axes.
   */
  void draw_cartesian_axes(const cv::Scalar& colour) const;

  /**
   * \brief Draws a line in cartesian coordinates.
   *
   * \param point1    The first endpoint of the line.
   * \param point2    The second endpoint of the line.
   * \param colour    The colour of the line.
   * \param thickness The thickness of the line.
   */
  void draw_cartesian_line(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Scalar& colour, int thickness = 1) const;

  /**
   * \brief Draws a circle in Cartesian coordinates.
   *
   * \param point     The centre of the circle.
   * \param colour    The colour of the point.
   * \param radius    The radius of the point.
   * \param thickness The thickness of the point (if thickness = -1 then the circle will be drawn filled).
   */
  void draw_cartesian_circle(const cv::Point2f& point, const cv::Scalar& colour, int radius = 2, int thickness = -1) const;

  /**
   * \brief Sets all the pixel values in the image to black.
   */
  void clear_figure() const;

  /**
   * \brief Gets the current height of the image.
   *
   * \return The height of the canvas being used for drawing.
   */
  size_t canvas_height() const;

  /**
   * \brief Draws a line in image coordinates.
   *
   * \param point1    The first endpoint of the line.
   * \param point2    The second endpoint of the line.
   * \param colour    The colour of the line.
   * \param thickness The thickness of the line.
   */
  void draw_image_line(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Scalar& colour, int thickness = 1) const;

  /**
   * \brief Draws a point in image coordinates.
   *
   * \param point      The image point.
   * \param colour     The colour of the point.
   * \param radius     The radius of the point.
   * \param thickness  The thickness of the point.
   */
  void draw_image_circle(const cv::Point2f& point, const cv::Scalar& colour, int radius = 5, int thickness = -1) const;

  /**
   * \brief Draws text in image coordinates.
   *
   * \param text       The test to be drawn.
   * \param position  The x-y position at which to draw the bottom-left corner of the text.
   * \param colour     The colour of the text.
   * \param scale      The size of the text.
   * \param thickness  The thickness of the text font.
   */
  void draw_image_text(const std::string& text, const cv::Point& position, const cv::Scalar& colour, double scale = 1.0, int thickness = 2) const;

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
  void save(const boost::optional<std::string>& path = boost::none);

  /**
   * \brief Displays the current image in a window.
   */
  void refresh() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Converts the specified Cartesian coordinate point to image coordinates.
   *
   * \param cartesianPoint   A point in Cartesian coordinates.
   * \return                 A point in image coordinates.
   */
  cv::Point2f cartesian_to_image(const cv::Point2f& cartesianPoint) const;

  /**
   * \brief Calculates the canvas position of a value in a line graph.
   * 
   * \param lineSeparation  The horizontal spacing between values in the line graph.
   * \param valueIndex      The index into the value array.
   * \param value           The value of the line graph.
   * \param maxValue        The maximum value in the line graph.
   * \return                The endpoint of the specified line in the graph (in canvas coordinates).
   */
  cv::Point line_graph_value_position_in_image_calculator(int lineSeparation, int valueIndex, float value, float maxValue) const;

  /**
   * \brief Converts a colour in RGB format to a colour in BGR format.
   *
   * \param colour  The specified colour in RGB format.
   * \return        The specified colour in BGR format.
   */
  cv::Scalar rgb_to_bgr(const cv::Scalar& colour) const;
};

#endif

