/**
 * tvgplot: OcvPlot.cpp
 */

#include <numeric>

#include <boost/lexical_cast.hpp>

#include "CvMatPlot.h"

//#################### CONSTRUCTOR #################### 
CvMatPlot::CvMatPlot(size_t figureNumber, std::string figureName, size_t imageWidth, size_t imageHeight)
: m_axesLength(5),
  m_imageWidth(imageWidth), m_imageHeight(imageHeight),
  m_canvas( cv::Mat::zeros(imageHeight, imageWidth, CV_8UC3) ),
  m_saveCounter(0)
{
  m_origin.x = imageWidth / 2.0f;
  m_origin.y = imageHeight / 2.0f;

  m_scaleWidth = float(imageWidth) / m_axesLength;
  m_scaleHeight = float(imageHeight) / m_axesLength;

  m_windowName = figureName + boost::lexical_cast<std::string>(figureNumber);
}

void CvMatPlot::draw_text(std::string text, cv::Point position, const Colour& colour, double scale, int thick){
    //position- bottom left corner of text in image
    putText(m_canvas, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thick, 8);
    show();
}

void CvMatPlot::Save(){
  char num[6];
  sprintf(num, "%05d", m_saveCounter++);
  std::string filename = std::string(m_windowName+ "-" + std::string(num) + ".ppm");
  imwrite(filename, m_canvas);
}

void CvMatPlot::show(){
  cv::imshow(m_windowName, m_canvas);
}

void CvMatPlot::clf(){
  m_canvas = cv::Mat::zeros(m_imageHeight, m_imageWidth, CV_8UC3);
  m_canvas = Colour(0,0,0);
  show();
}

void CvMatPlot::draw_point(cv::Point2f point, const Colour& colour, int radius, int thickness){
 
    cv::circle( m_canvas,
         point,
         radius, 
         colour,
         thickness);
}

void CvMatPlot::canvas_point(cv::Point2f point, const Colour& colour, int radius, int thickness)
{
  draw_point(axes2image(point), colour, radius, thickness);
}

void CvMatPlot::draw_point(std::vector<cv::Point2f> p, std::vector<int> col){
     for (int i=0;i<p.size();i++){
            draw_point(p[i],col[i]);
     }
}

void CvMatPlot::draw_line(cv::Point2f p1, cv::Point2f p2, const Colour& colour, int thick){
    
    cv::line( m_canvas,
        p1,
        p2,
        colour,
        thick);  
    
}

void CvMatPlot::draw_unit_circle(const Colour& colour){
    cv::Point2f o(0.0);
    int radius = 1;
    
    draw_point(axes2image(o), colour, radius*m_scaleWidth, 1);
}

void CvMatPlot::draw_axis(const Colour& colour){
    cv::Point2f xleft(-5,0), xright(5,0);
    cv::Point2f ydown(0,-5), yup(0,5);
    draw_line(axes2image(xleft), axes2image(xright), colour);
    draw_line(axes2image(ydown), axes2image(yup), colour);
}

void CvMatPlot::draw_hyperplane(cv::Point2f w, float b, const Colour& colour){
    
    draw_axis(colour);
    draw_unit_circle(colour);
    
    cv::Point2f o(0,0);
    draw_line(axes2image(o), axes2image(w),colour);
    cv::Point2f bias(0,-b/w.y);
    //PRT(b); PRT(w); PRT(bias);
    draw_point(axes2image(bias),colour);
    int a=3;
    cv::Point2f h1(-a,(-w.x*(-a) - b)/w.y);
    cv::Point2f h2(+a,(-w.x*(+a) - b)/w.y);
    cv::Point2f m1(-a,(-w.x*(-a) - b +1)/w.y);
    cv::Point2f m2(+a,(-w.x*(+a) - b +1)/w.y);
    cv::Point2f m3(-a,(-w.x*(-a) - b -1)/w.y);
    cv::Point2f m4(+a,(-w.x*(+a) - b -1)/w.y);
    draw_line(axes2image(h1),axes2image(h2),colour,2);
    draw_line(axes2image(m1),axes2image(m2),colour);
    draw_line(axes2image(m3),axes2image(m4),colour);
    
    show();
}

void CvMatPlot::draw_bars(const std::vector<float>& bars, const Colour& colour){
    if(bars.size()>0){
    int length = bars.size();
    int bar_w = cvRound( (double) m_imageWidth/length );

    float maxval = *std::max_element(bars.begin(),bars.end());
    
    for( int j = 1; j < length; j++ )
    {
        cv::line( m_canvas, cv::Point( bar_w*(j-1), m_imageHeight - cvRound(m_imageHeight* (bars[j-1]/maxval)) ) ,
                        cv::Point( bar_w*(j), m_imageHeight - cvRound(m_imageHeight* (bars[j]/maxval)) ),
                        colour, 2, 8, 0);
    }


    show();
    }else{
      std::cout << "CvMatPlot::draw_bar: v.size() is empty..\n";
    }
}

cv::Point2f CvMatPlot::axes2image(const cv::Point2f axesPoint){
    
  //scale
  cv::Point2f imagePoint;
  imagePoint.x = axesPoint.x*m_scaleWidth;
  imagePoint.y = axesPoint.y*m_scaleHeight;
 
  //translate
  imagePoint.x += m_origin.x;
  imagePoint.y += m_origin.y;
  imagePoint.y = m_imageHeight - imagePoint.y;
  
  return imagePoint;
}

