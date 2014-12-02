/**
 * rafl: UnitCircleExampleGenerator.h
 */

#ifndef H_RAFL_UNITCIRCLEEXAMPLEGENERATOR
#define H_RAFL_UNITCIRCLEEXAMPLEGENERATOR

#include <cmath>
#include <set>
#include <stdexcept>

#include <tvgutil/RandomNumberGenerator.h>

#include "Example.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to generate examples from class distributions that are equally spaced around the unit circle.
 */
template <typename Label>
class UnitCircleExampleGenerator
{
  //#################### PRIVATE TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct can be used to specify the parameters for one of the class distributions.
   *
   * As currently implemented, the distributions are 2D Gaussians with diagonal covariance (ask Michael/Wikipedia if you don't know what this means!).
   * For each class, we have a reference point that is located at a particular angle on the unit circle. The distribution is then centred around this
   * reference point, with differing standard deviations in the x and y directions.
   */
  struct ClassParameters
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

    /** The angle at which this class is centered on the unit circle. */
    float m_angle;

    /** The standard deviation of the distribution in the x direction. */
    float m_xSTD;

    /** The standard deviation of the distribution in the y direction. */
    float m_ySTD;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~

    ClassParameters(float angle, float xSTD, float ySTD)
    : m_angle(angle), m_xSTD(xSTD), m_ySTD(ySTD)
    {}
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The parameters for the various class distributions. */
  std::map<Label,ClassParameters> m_classParameters;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator m_gen;

  //#################### CONSTRUCTORS ####################
  /**
  * \brief Constructs a unit circle example generator.
  *
  * \param classLabels  The labels of the classes for which we want to generate distributions.
  * \param seed         The seed for the random number generator we will be using.
  */
public:
  UnitCircleExampleGenerator(const std::set<Label>& classLabels, unsigned int seed)
  : m_gen(seed)
  {
    // Construct the distributions for the various classes.
    const float ROTATION_PER_CLASS = static_cast<float>(2.0 * M_PI / classLabels.size());
    const float STD_LOWER_BOUND = 0.1f;
    const float STD_UPPER_BOUND = 0.5f;
    int i = 0; 
    for(std::set<Label>::const_iterator it = classLabels.begin(), iend = classLabels.end(); it != iend; ++it)
    {
      float angle = i * ROTATION_PER_CLASS;
      float xSTD = m_gen.generate_real_from_uniform<>(STD_LOWER_BOUND, STD_UPPER_BOUND);
      float ySTD = m_gen.generate_real_from_uniform<>(STD_LOWER_BOUND, STD_UPPER_BOUND);
      m_classParameters.insert(std::make_pair(*it, ClassParameters(angle, xSTD, ySTD)));
      ++i;
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /*
   * \brief Generates examples from a subset of the classes for which the example generator is configured.
   *
   * \param sampleClassLabels       The labels of the classes from which to generate examples.
   * \param numberOfSamplesPerClass The number of examples to generate per class.
   * \return                        The generated examples.
   */
  std::vector<Example_CPtr> generate_examples(const std::set<Label>& sampleClassLabels, size_t numberOfSamplesPerClass)
  {
    std::vector<Example_CPtr> result;
    for(size_t i = 0; i < numberOfSamplesPerClass; ++i)
    {
      for(typename std::set<Label>::const_iterator it = sampleClassLabels.begin(), iend = sampleClassLabels.end(); it != iend; ++it)
      {
        std::map<Label,ClassParameters>::const_iterator jt = m_classParameters.find(*it);
        if(jt == m_classParameters.end()) throw std::runtime_error("The example generator is not configured to generate examples for the specified class");
        result.push_back(Example_CPtr(new Example<Label>(make_sample_descriptor(jt->second), *it)));
      }
    }
    return result;
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a 2D descriptor corresponding to a sample from the class with the specified parameters.
   *
   * \param classParameters The parameters of the class from which to sample.
   * \return                The descriptor for the sample.
   */
  Descriptor_CPtr make_sample_descriptor(const ClassParameters& classParameters)
  {
    Descriptor_Ptr d(new Descriptor(2));
    (*d)[0] = m_gen.generate_from_gaussian(cos(classParameters.m_angle), classParameters.m_xSTD);
    (*d)[1] = m_gen.generate_from_gaussian(sin(classParameters.m_angle), classParameters.m_ySTD);
    return d;
  }
};

}

#endif
