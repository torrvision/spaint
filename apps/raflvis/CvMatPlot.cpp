/**
 * tvgplot: OcvPlot.cpp
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

void CvMatPlot::cartesian_axes(const Colour& colour)
{
  cv::Point2f xleft(-5,0), xright(5,0);
  cv::Point2f ydown(0,-5), yup(0,5);
  image_line(axes2image(xleft), axes2image(xright), colour);
  image_line(axes2image(ydown), axes2image(yup), colour);
}

void CvMatPlot::cartesian_point(cv::Point2f point, const Colour& colour, int radius, int thickness)
{
  image_point(axes2image(point), colour, radius, thickness);
}

void CvMatPlot::clf(){
  m_canvas = cv::Mat::zeros(m_imageHeight, m_imageWidth, CV_8UC3);
  m_canvas = Colour(0,0,0);
}

size_t CvMatPlot::height() const
{
  return m_imageHeight;
}

void CvMatPlot::image_line(cv::Point2f p1, cv::Point2f p2, const Colour& colour, int thick)
{    
  cv::line(m_canvas, p1, p2, rgb2bgr(colour), thick);     
}

void CvMatPlot::image_point(const cv::Point2f& point, const Colour& colour, int radius, int thickness){
  cv::circle(m_canvas, point, radius, rgb2bgr(colour), thickness);
}

void CvMatPlot::image_text(std::string text, cv::Point position, const Colour& colour, double scale, int thick){
    //position- bottom left corner of text in image
    putText(m_canvas, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thick, 8);
}

void CvMatPlot::line_graph(const std::vector<float>& values, const Colour& colour)
{
  if(values.empty())
    throw std::runtime_error("The values vector is empty.");

  int length = values.size();
  int bar_w = cvRound( (double) m_imageWidth/length );

  float maxval = *std::max_element(values.begin(),values.end());
  
  for( int j = 1; j < length; j++ )
  {
    cv::line( m_canvas, 
      cv::Point(bar_w*(j-1), m_imageHeight - cvRound(m_imageHeight* (values[j-1]/maxval))),
      cv::Point(bar_w*(j), m_imageHeight - cvRound(m_imageHeight* (values[j]/maxval))),
      rgb2bgr(colour), 2, 8, 0);
  }

}

void CvMatPlot::save(const std::string& path)
{
  char num[6]; sprintf(num, "%05d", m_saveCounter++);
  std::string filename = std::string(m_windowName+ "-" + std::string(num) + ".ppm");
  imwrite(path + "/" + filename, m_canvas);
}

void CvMatPlot::show(){
  cv::imshow(m_windowName, m_canvas);
}

//#################### PRIVATE MEMBER FUNCTIONS #################### 

cv::Point2f CvMatPlot::axes2image(const cv::Point2f axesPoint){
    
  //scale
  cv::Point2f imagePoint;
  imagePoint.x = axesPoint.x*m_scaleWidth;
  imagePoint.y = axesPoint.y*m_scaleHeight;
 
  //translate
  imagePoint.x += m_cartesianOriginInImage.x;
  imagePoint.y += m_cartesianOriginInImage.y;
  imagePoint.y = m_imageHeight - imagePoint.y;
  
  return imagePoint;
}

cv::Scalar CvMatPlot::rgb2bgr(const cv::Scalar& colour)
{
  return cv::Scalar(colour.val[2],colour.val[1],colour.val[0]);
}

