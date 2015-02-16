/**
 * raflvis: CvMatPlot.cpp
 */

#include <numeric>
#include <stdexcept>

#include <boost/lexical_cast.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "CvMatPlot.h"

//#################### CONSTRUCTOR ####################

CvMatPlot::CvMatPlot(size_t figureNumber, std::string figureName, size_t imageWidth, size_t imageHeight, int axesLength)
: m_axesLength(axesLength),
  m_canvas( cv::Mat::zeros(imageHeight, imageWidth, CV_8UC3) ),
  m_imageHeight(imageHeight),
  m_imageWidth(imageWidth),
  m_saveCounter(0),
  m_scaleHeight(static_cast<float>(imageHeight/axesLength)),
  m_scaleWidth(static_cast<float>(imageWidth/axesLength)),
  m_windowName(figureName + boost::lexical_cast<std::string>(figureNumber))
{
  if(axesLength <= 0)
    throw std::runtime_error("The axes lengths must be greater than zero.");

  // Sets the origin to the center of the image.
  m_cartesianOriginInImage.x = imageWidth / 2.0f;
  m_cartesianOriginInImage.y = imageHeight / 2.0f;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CvMatPlot::cartesian_axes(const cv::Scalar& colour) const
{
  int xMin = -1 * (m_axesLength / 2.0f);
  int xMax = m_axesLength / 2.0f;
  int yMin = xMin;
  int yMax = xMax;

  image_line(axes2image(cv::Point2f(xMin, 0)), axes2image(cv::Point2f(xMax, 0)), colour);
  image_line(axes2image(cv::Point2f(0, yMin)), axes2image(cv::Point2f(0, yMax)), colour);
}

void CvMatPlot::cartesian_point(cv::Point2f point, const cv::Scalar& colour, int radius, int thickness) const
{
  image_point(axes2image(point), colour, radius, thickness);
}

void CvMatPlot::clf() const
{
  m_canvas = cv::Mat::zeros(m_imageHeight, m_imageWidth, CV_8UC3);
  m_canvas = cv::Scalar(0,0,0);
}

size_t CvMatPlot::height() const
{
  return m_imageHeight;
}

void CvMatPlot::image_line(cv::Point2f p1, cv::Point2f p2, const cv::Scalar& colour, int thick) const
{
  cv::line(m_canvas, p1, p2, rgb2bgr(colour), thick);
}

void CvMatPlot::image_point(const cv::Point2f& point, const cv::Scalar& colour, int radius, int thickness) const
{
  cv::circle(m_canvas, point, radius, rgb2bgr(colour), thickness);
}

void CvMatPlot::image_text(std::string text, cv::Point position, const cv::Scalar& colour, double scale, int thick) const
{
    // The variable position refers to the bottom left corner of text in the image.
    putText(m_canvas, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thick);
}

void CvMatPlot::line_graph(const std::vector<float>& values, const cv::Scalar& colour) const
{
  if(values.empty())
    throw std::runtime_error("The values vector is empty.");

  int valuesSize = values.size();
  int lineSeparation = cvRound( static_cast<float>(m_imageWidth) / valuesSize );

  float maxval = *std::max_element(values.begin(),values.end());

  const int lineThickness = 2;

  for(int j = 1; j < valuesSize; ++j)
  {
    cv::Point lineBegin(lineSeparation * (j-1), m_imageHeight - cvRound(m_imageHeight * (values[j - 1] / maxval)));
    cv::Point lineEnd(lineSeparation * j, m_imageHeight - cvRound(m_imageHeight * (values[j] / maxval)));
    image_line(lineBegin, lineEnd, rgb2bgr(colour), lineThickness);
  }

}

void CvMatPlot::save(const std::string& path)
{
  char num[6]; sprintf(num, "%05d", m_saveCounter++);
  std::string filename = std::string(m_windowName+ "-" + std::string(num) + ".ppm");
  imwrite(path + "/" + filename, m_canvas);
}

void CvMatPlot::show() const
{
  cv::imshow(m_windowName, m_canvas);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

cv::Point2f CvMatPlot::axes2image(const cv::Point2f axesPoint) const
{
  // Scale
  cv::Point2f imagePoint(axesPoint.x * m_scaleWidth, axesPoint.y * m_scaleHeight);

  // Translation
  imagePoint.x += m_cartesianOriginInImage.x;
  imagePoint.y += m_cartesianOriginInImage.y;
  imagePoint.y = m_imageHeight - imagePoint.y;

  return imagePoint;
}

cv::Scalar CvMatPlot::rgb2bgr(const cv::Scalar& colour) const
{
  return cv::Scalar(colour.val[2], colour.val[1], colour.val[0]);
}

