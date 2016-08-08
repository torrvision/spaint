/**
 * tvgplot: PlotWindow.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGPLOT_PLOTWINDOW
#define H_TVGPLOT_PLOTWINDOW

#include <string>

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace tvgplot {

/**
 * \brief An instance of this class represents a window into which we can plot shapes and/or a graph.
 */
class PlotWindow
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The absolute length of the visible axis in both the x and y directions. */
  float m_axesLength;

  /** The canvas on which to draw the shapes or graph. */
  mutable cv::Mat m_canvas;

  /** The height of the canvas (in pixels). */
  int m_canvasHeight;

  /** The width of the canvas (in pixels). */
  int m_canvasWidth;

  /** The origin of the Cartesian coordinate system (in canvas coordinates). */
  cv::Point2f m_cartesianOriginInCanvas;

  /** A counter recording how many times the canvas has been saved (this is used to name the files when saving a stream of images to disk). */
  int m_saveCounter;

  /** The vertical scaling factor by which to multiply when converting Cartesian coordinates to canvas coordinates. */
  float m_scaleHeight;

  /** The horizontal scaling factor by which to multiply when converting Cartesian coordinates to canvas coordinates. */
  float m_scaleWidth;

  /** The name to display in the window. */
  std::string m_windowName;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a window into which we can plot shapes and/or a graph.
   *
   * \param windowName      The name to display in the window.
   * \param canvasWidth     The width of the canvas (in pixels).
   * \param canvasHeight    The height of the canvas (in pixels).
   * \param axesLength      The absolute length of the visible axis in both the x and y directions.
   */
  explicit PlotWindow(const std::string& windowName, int canvasWidth = 700, int canvasHeight = 700, float axesLength = 5);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the height of the canvas (in pixels).
   *
   * \return The height of the canvas (in pixels).
   */
  int canvas_height() const;

  /**
   * \brief Sets the values of all the pixels in the canvas to black.
   */
  void clear_figure() const;

  /**
   * \brief Draws a circle in canvas coordinates.
   *
   * \param point      The centre of the circle.
   * \param colour     The colour of the circle.
   * \param radius     The radius of the circle.
   * \param thickness  The thickness of the circle (if thickness = -1 then the circle will be drawn filled).
   */
  void draw_canvas_circle(const cv::Point2f& centre, const cv::Scalar& colour, int radius = 5, int thickness = -1) const;

  /**
   * \brief Draws a line in canvas coordinates.
   *
   * \param point1    The first endpoint of the line.
   * \param point2    The second endpoint of the line.
   * \param colour    The colour of the line.
   * \param thickness The thickness of the line.
   */
  void draw_canvas_line(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Scalar& colour, int thickness = 1) const;

  /**
   * \brief Draws text in canvas coordinates.
   *
   * \param text       The text to be drawn.
   * \param position   The x-y position of the bottom-left corner of the text.
   * \param colour     The colour of the text.
   * \param scale      The size of the text.
   * \param thickness  The thickness of the text font.
   */
  void draw_canvas_text(const std::string& text, const cv::Point& position, const cv::Scalar& colour, double scale = 1.0, int thickness = 2) const;

  /**
   * \brief Draws the Cartesian axes.
   *
   * \param colour  The colour of the axes.
   */
  void draw_cartesian_axes(const cv::Scalar& colour) const;

  /**
   * \brief Draws a circle in Cartesian coordinates.
   *
   * \param centre    The centre of the circle.
   * \param colour    The colour of the circle.
   * \param radius    The radius of the circle.
   * \param thickness The thickness of the circle (if thickness = -1 then the circle will be drawn filled).
   */
  void draw_cartesian_circle(const cv::Point2f& centre, const cv::Scalar& colour, int radius = 2, int thickness = -1) const;

  /**
   * \brief Draws a line in Cartesian coordinates.
   *
   * \param point1    The first endpoint of the line.
   * \param point2    The second endpoint of the line.
   * \param colour    The colour of the line.
   * \param thickness The thickness of the line.
   */
  void draw_cartesian_line(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Scalar& colour, int thickness = 1) const;

  /**
   * \brief Draws a line graph.
   *
   * \param values  The values making up the line graph.
   * \param colour  The colour of the line graph.
   */
  void draw_line_graph(const std::vector<float>& values, const cv::Scalar& colour) const;

  /**
   * \brief Refreshes the canvas.
   */
  void refresh() const;

  /**
   * \brief Saves the current canvas to a file.
   *
   * \param path  An optional path to a directory in which to place the file (if boost::none, the current working directory is assumed).
   */
  void save(const boost::optional<std::string>& path = boost::none);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Converts the specified Cartesian coordinate point to canvas coordinates.
   *
   * \param cartesianPoint   A point in Cartesian coordinates.
   * \return                 A point in canvas coordinates.
   */
  cv::Point2f cartesian_to_canvas(const cv::Point2f& cartesianPoint) const;

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

}

#endif
