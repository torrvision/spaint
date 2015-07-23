/**
 * raflvis: PlotWindow.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "PlotWindow.h"

#include <numeric>
#include <stdexcept>

#include <boost/lexical_cast.hpp>

#include <opencv2/imgproc/imgproc.hpp>

//#################### CONSTRUCTORS ####################

PlotWindow::PlotWindow(const std::string& windowName, int canvasWidth, int canvasHeight, float axesLength)
: m_axesLength(axesLength),
  m_canvas(cv::Mat::zeros(canvasHeight, canvasWidth, CV_8UC3)),
  m_canvasHeight(canvasHeight),
  m_canvasWidth(canvasWidth),
  m_saveCounter(0),
  m_scaleHeight(static_cast<float>(canvasHeight) / axesLength),
  m_scaleWidth(static_cast<float>(canvasWidth) / axesLength),
  m_windowName(windowName)
{
  if(axesLength <= 0) throw std::runtime_error("The lengths of the axes must be greater than zero");

  // Place the origin of the Cartesian coordinate system in the centre of the canvas.
  m_cartesianOriginInCanvas.x = canvasWidth / 2.0f;
  m_cartesianOriginInCanvas.y = canvasHeight / 2.0f;

  // Create a window for display.
  cv::namedWindow(m_windowName, cv::WINDOW_AUTOSIZE);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int PlotWindow::canvas_height() const
{
  return m_canvasHeight;
}

void PlotWindow::clear_figure() const
{
  m_canvas = cv::Scalar(0,0,0);
}

void PlotWindow::draw_canvas_circle(const cv::Point2f& centre, const cv::Scalar& colour, int radius, int thickness) const
{
  cv::circle(m_canvas, centre, radius, rgb_to_bgr(colour), thickness);
}

void PlotWindow::draw_canvas_line(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Scalar& colour, int thickness) const
{
  cv::line(m_canvas, p1, p2, rgb_to_bgr(colour), thickness);
}

void PlotWindow::draw_canvas_text(const std::string& text, const cv::Point& position, const cv::Scalar& colour, double scale, int thickness) const
{
    putText(m_canvas, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thickness);
}

void PlotWindow::draw_cartesian_axes(const cv::Scalar& colour) const
{
  float axisMax = static_cast<float>(m_axesLength) / 2.0f;
  float axisMin = -axisMax;

  draw_cartesian_line(cv::Point2f(axisMin, 0), cv::Point2f(axisMax, 0), colour);
  draw_cartesian_line(cv::Point2f(0, axisMin), cv::Point2f(0, axisMax), colour);
}

void PlotWindow::draw_cartesian_circle(const cv::Point2f& centre, const cv::Scalar& colour, int radius, int thickness) const
{
  draw_canvas_circle(cartesian_to_canvas(centre), colour, radius, thickness);
}

void PlotWindow::draw_cartesian_line(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Scalar& colour, int thickness) const
{
  draw_canvas_line(cartesian_to_canvas(p1), cartesian_to_canvas(p2), colour, thickness);
}

void PlotWindow::draw_line_graph(const std::vector<float>& values, const cv::Scalar& colour) const
{
  if(values.empty()) throw std::runtime_error("Cannot draw the line graph of an empty set of values");

  int valuesSize = static_cast<int>(values.size());
  int lineSeparation = cvRound(static_cast<float>(m_canvasWidth) / valuesSize);

  float maxval = *std::max_element(values.begin(), values.end());

  const cv::Scalar col = rgb_to_bgr(colour);
  const int lineThickness = 2;

  cv::Point lineBegin(0,0), lineEnd(0,0);
  for(int j = 0; j < valuesSize; ++j)
  {
    lineBegin = line_graph_value_position_in_image_calculator(lineSeparation, j, values[j], maxval);
    if(j > 0) draw_canvas_line(lineBegin, lineEnd, col, lineThickness);
    lineEnd = lineBegin;
  }
}

void PlotWindow::refresh() const
{
  cv::imshow(m_windowName, m_canvas);
}

void PlotWindow::save(const boost::optional<std::string>& path)
{
  std::string filename = m_windowName + "-" + boost::lexical_cast<std::string>(m_saveCounter++) + ".ppm";

  if(path)
  {
    imwrite(*path + "/" + filename, m_canvas);
  }
  else
  {
    imwrite(filename, m_canvas);
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

cv::Point2f PlotWindow::cartesian_to_canvas(const cv::Point2f& cartesianPoint) const
{
  // Scale
  cv::Point2f imagePoint(cartesianPoint.x * m_scaleWidth, cartesianPoint.y * m_scaleHeight);

  // Translation
  imagePoint += m_cartesianOriginInCanvas;

  // Vertical Flipping
  imagePoint.y = m_canvasHeight - imagePoint.y;

  return imagePoint;
}

cv::Point PlotWindow::line_graph_value_position_in_image_calculator(int lineSeparation, int valueIndex, float value, float maxValue) const
{
  return cv::Point(lineSeparation * valueIndex, m_canvasHeight - cvRound(m_canvasHeight * (value / maxValue)));
}

cv::Scalar PlotWindow::rgb_to_bgr(const cv::Scalar& colour) const
{
  return cv::Scalar(colour.val[2], colour.val[1], colour.val[0]);
}
