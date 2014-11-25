/**
 * rafl: UnitCircleExampleGenerator.h
 */

#ifndef H_RAFL_UNITCIRCLEEXAMPLEGENERATOR
#define H_RAFL_UNITCIRCLEEXAMPLEGENERATOR

#include <cmath>
#include <set>

#include <Eigen/Dense>
using Eigen::ArrayXf;
using Eigen::Vector2f;
using Eigen::VectorXf;
using Eigen::Matrix2f;

#include "ExampleUtil.h"

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
  Vector2f m_referencePoint;

  Matrix2f m_2dRotationMatrix;

  //Defines the angle with which to rotate the reference unit vector. 
  float m_rotationPerClass;
 
  std::map<Label,int> m_uniqueClassLabels;

  //#################### CONSTRUCTORS ####################
public:
  explicit UnitCircleExampleGenerator(std::map<Label,int> uniqueClassLabels)
  : m_uniqueClassLabels(uniqueClassLabels)
  {
    m_referencePoint(0) = 0.0f;
    m_referencePoint(1) = 1.0f;

    m_rotationPerClass = 2.0f*M_PI/m_uniqueClassLabels.size();
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
     
    for(size_t i = 0; i < numberOfSamplesPerClass; ++i)
    {
      for(typename std::set<Label>::const_iterator it = labelSet.begin(), iend = labelSet.end(); it != iend; ++it)
      {
        float angle = m_uniqueClassLabels[*it]*m_rotationPerClass;
        m_2dRotationMatrix = Eigen::Rotation2Df(angle);
        
        Vector2f rotatedPoint = (m_referencePoint.transpose()*m_2dRotationMatrix).transpose();
  
        float a = rotatedPoint(0); // + Gaussian Noise
        float b = rotatedPoint(1); // + Gaussian Noise
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
