/**
 * raflvis: CvPlotter.cpp
 */

#include <numeric>
#include <stdexcept>

#include <boost/lexical_cast.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "CvPlotter.h"

//#################### CONSTRUCTOR ####################

CvPlotter::CvPlotter(std::string windowName, size_t imageWidth, size_t imageHeight, int axesLength)
: m_axesLength(axesLength),
  m_canvas(cv::Mat::zeros(imageHeight, imageWidth, CV_8UC3)),
  m_imageHeight(imageHeight),
  m_imageWidth(imageWidth),
  m_saveCounter(0),
  m_scaleHeight(static_cast<float>(imageHeight/axesLength)),
  m_scaleWidth(static_cast<float>(imageWidth/axesLength)),
  m_windowName(windowName)
{
  if(axesLength <= 0)
    throw std::runtime_error("The axes lengths must be greater than zero.");

  // Sets the origin to the centre of the image.
  m_cartesianOriginInImage.x = imageWidth / 2.0f;
  m_cartesianOriginInImage.y = imageHeight / 2.0f;

  // Create a window for display.
  cv::namedWindow(m_windowName, cv::WINDOW_AUTOSIZE);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CvPlotter::cartesian_axes(const cv::Scalar& colour) const
{
  int xMin = -1 * (m_axesLength / 2.0f);
  int xMax = m_axesLength / 2.0f;
  int yMin = xMin;
  int yMax = xMax;

  image_line(axes2image(cv::Point2f(xMin, 0)), axes2image(cv::Point2f(xMax, 0)), colour);
  image_line(axes2image(cv::Point2f(0, yMin)), axes2image(cv::Point2f(0, yMax)), colour);
}

void CvPlotter::cartesian_point(cv::Point2f point, const cv::Scalar& colour, int radius, int thickness) const
{
  image_point(axes2image(point), colour, radius, thickness);
}

void CvPlotter::clf() const
{
  m_canvas = cv::Mat::zeros(m_imageHeight, m_imageWidth, CV_8UC3);
  m_canvas = cv::Scalar(0,0,0);
}

size_t CvPlotter::height() const
{
  return m_imageHeight;
}

void CvPlotter::image_line(cv::Point2f p1, cv::Point2f p2, const cv::Scalar& colour, int thick) const
{
  cv::line(m_canvas, p1, p2, rgb2bgr(colour), thick);
}

void CvPlotter::image_point(const cv::Point2f& point, const cv::Scalar& colour, int radius, int thickness) const
{
  cv::circle(m_canvas, point, radius, rgb2bgr(colour), thickness);
}

void CvPlotter::image_text(const std::string& text, const cv::Point& position, const cv::Scalar& colour, double scale, int thick) const
{
    // The variable position refers to the bottom left corner of text in the image.
    putText(m_canvas, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thick);
}

void CvPlotter::line_graph(const std::vector<float>& values, const cv::Scalar& colour) const
{
  if(values.empty()) throw std::runtime_error("The values vector is empty.");

  int valuesSize = static_cast<int>(values.size());
  int lineSeparation = cvRound( static_cast<float>(m_imageWidth) / valuesSize );

  float maxval = *std::max_element(values.begin(), values.end());

  const int lineThickness = 2;

  for(int j = 1; j < valuesSize; ++j)
  {
    cv::Point lineBegin = calculateLineGraphValuePositionInImage(lineSeparation, j - 1, m_imageHeight, values[j - 1], maxval);
    cv::Point lineEnd = calculateLineGraphValuePositionInImage(lineSeparation, j, m_imageHeight, values[j], maxval);
    image_line(lineBegin, lineEnd, rgb2bgr(colour), lineThickness);
  }

}

void CvPlotter::save(const boost::optional<std::string>& path)
{
  std::string filename = m_windowName + "-" + boost::lexical_cast<std::string>(m_saveCounter++) + ".ppm";

  if(path){
    imwrite(*path + "/" + filename, m_canvas);
  }
  else{
    imwrite(filename, m_canvas);
  }
}

void CvPlotter::show() const
{
  cv::imshow(m_windowName, m_canvas);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

cv::Point2f CvPlotter::axes2image(const cv::Point2f& cartesianPoint) const
{
  // Scale
  cv::Point2f imagePoint(cartesianPoint.x * m_scaleWidth, cartesianPoint.y * m_scaleHeight);

  // Translation
  imagePoint += m_cartesianOriginInImage;

  // Vertical Flipping
  imagePoint.y = m_imageHeight - imagePoint.y;

  return imagePoint;
}

cv::Point CvPlotter::calculateLineGraphValuePositionInImage(int lineSeparation, int valueIndex, int imageHeight, float value, float maxValue) const
{
  return cv::Point(lineSeparation * valueIndex, imageHeight - cvRound(imageHeight * (value / maxValue)));
}

cv::Scalar CvPlotter::rgb2bgr(const cv::Scalar& colour) const
{
  return cv::Scalar(colour.val[2], colour.val[1], colour.val[0]);
}

