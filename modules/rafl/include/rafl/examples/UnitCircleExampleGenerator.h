/**
 * rafl: UnitCircleExampleGenerator.h
 */

#ifndef H_RAFL_UNITCIRCLEEXAMPLEGENERATOR
#define H_RAFL_UNITCIRCLEEXAMPLEGENERATOR

#include <Eigen/Dense>
using Eigen::ArrayXf;
using Eigen::Vector2f;
using Eigen::VectorXf;
using Eigen::Matrix2f;

#include "ExampleUtil.h"

#include <set>

namespace rafl {

  /**
   * \brief This class generates examples drawn from points equally spaced around the unit circle.
   */
template <typename Label>
class UnitCircleExampleGenerator
{
  //#################### PRIVATE TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  float m_d1;
  float m_d2;

  //Defines the angle with which to rotate the unit vector [d1 d2]' 
  float m_rotationPerClass;
 
  std::map<Label,int> m_uniqueClassLabels;

  float m_pi;

  //#################### CONSTRUCTORS ####################
public:
  explicit UnitCircleExampleGenerator(std::map<Label,int> uniqueClassLabels)
  : m_d1(0.0f), m_d2(1.0f), m_pi(3.14f), m_uniqueClassLabels(uniqueClassLabels)
  {
    m_rotationPerClass = 2.0f*m_pi/m_uniqueClassLabels.size();
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
/*
 * \brief Creates a vector of examples with the same number of examples per class
 *
 * \param labelSet                  The set of labels from which to draw the samples
 * \param numberOfSamplesPerClass   The number of samples to create per class
 * \return                          The set of generated examples
 */
  std::vector<Example_CPtr> generate_examples_in_set(const std::set<Label>& labelSet, size_t numberOfSamplesPerClass)
  {
    std::vector<Example_CPtr> exampleSet;
    //std::vector<Label> labelSetVector(labelSet.begin(), labelSet.end());

    //typename std::set<Label>::const_iterator set_iterator = labelSet.begin();
     
    for(size_t i = 0; i < numberOfSamplesPerClass; ++i)
    {
      for(typename std::set<Label>::const_iterator it = labelSet.begin(), iend = labelSet.end(); it != iend; ++it)
      {
        float angle = m_uniqueClassLabels[*it]*m_rotationPerClass;
        float r11 = cos(angle);
        float r12 = -sin(angle);
        float r21 = sin(angle);
        float r22 = cos(angle);
          
        //Rotation matrix (j*(2*pi/numberOfSamplesPerClass)); R = [cos(x) -sin(x); sin(x) cos(x)];
        float a = r11*m_d1 + r12*m_d2;
        float b = r21*m_d1 + r22*m_d2;
        std::cout << "(" << a << "," << b << ")\n";
        
        Example_CPtr e(new Example<Label>(ExampleUtil::make_const_2d_descriptor(a,b), *it));
        exampleSet.push_back( e );
      }
    }
    
    return exampleSet;
  }
};

}

#endif
