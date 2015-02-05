/**
 * tvgplot: OcvPlot.h
 */

#ifndef H_TVGPLOT_OCVPLOT
#define H_TVGPLOT_OCVPLOT

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * \brief An instance of this class allows 2D plotting in a OpenCV matrix.
 */
class CvMatPlot
{
  //typedef tvgutil::RGBAColourUC Colour;
  typedef cv::Scalar Colour;

private:
  int m_saveCounter;
  size_t m_figureNumber;
  std::string m_windowName;
  float m_axesLength;
  cv::Mat m_canvas;
  cv::Point2f m_origin;
  float m_scaleWidth, m_scaleHeight;
  size_t m_imageWidth, m_imageHeight;

public:
  CvMatPlot(size_t figureNumber, std::string figureName = "Drawing pad", size_t imageWidth = 700, size_t imageHeight = 700);

  void draw_point(cv::Point2f point, const Colour& colour, int radius=5, int thickness=-1);
  void draw_point(std::vector<cv::Point2f> point, std::vector<int> col);
  void canvas_point(cv::Point2f point, const Colour& colour, int radius, int thickness);
  
  void draw_text(std::string text, cv::Point position, const Colour& colour, double scale = 1.0, int thickness=2);
  void draw_axis(const Colour& colour);
  void draw_unit_circle(const Colour& colour);
  void draw_line(cv::Point2f point1, cv::Point2f point2, const Colour& colour, int thickness=1);
  void draw_hyperplane(cv::Point2f normal, float bias, const Colour& colour);
  void draw_bars(const std::vector<float>& bars, const Colour& colour);
  void clf();
  void Save();
  void show();
  
private:
  cv::Point2f axes2image(const cv::Point2f axesPoint);
};

#endif

