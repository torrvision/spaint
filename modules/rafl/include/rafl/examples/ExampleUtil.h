/**
 * rafl: ExampleUtil.h
 */

#ifndef H_RAFL_EXAMPLEUTIL
#define H_RAFL_EXAMPLEUTIL

#include "../base/ProbabilityMassFunction.h"
#include "Example.h"

#include <set>

namespace rafl {

/**
 * \brief This class contains utility functions that work on examples.
 */
class ExampleUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  
  /**
   * \brief Create a constant 2d descriptor
   * 
   * \param x   The x component of the descriptor
   * \param y   The y component of the descriptor
   * \return    The const descriptor
   */
  static Descriptor_CPtr make_const_2d_descriptor(float x, float y)
  {
    Descriptor_Ptr d(new Descriptor(2));
    (*d)[0] = x;
    (*d)[1] = y;
    return d;
  }
  static Descriptor_Ptr make_2d_descriptor(float x, float y)
  {
    Descriptor_Ptr d(new Descriptor(2));
    (*d)[0] = x;
    (*d)[1] = y;
    return d;
  }
  
  /**
   * \brief Calculates the entropy of the label distribution of a set of examples.
   *
   * \param examples  The examples for whose label distribution we want to calculate the entropy.
   * \return          The entropy of the examples' label distribution.
   */
  template <typename Label>
  static float calculate_entropy(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    return make_pmf(examples).calculate_entropy();
  }

  /**
   * \brief Calculates the entropy of a label distribution represented by the specified histogram.
   *
   * \param histogram The histogram.
   * \return          The entropy of the label distribution represented by the histogram.
   */
  template <typename Label>
  static float calculate_entropy(const Histogram<Label>& histogram)
  {
    return ProbabilityMassFunction<Label>(histogram).calculate_entropy();
  }

  /**
   * \brief Makes a histogram from the label distribution of a set of examples.
   *
   * \param examples  The examples from whose label distribution we want to make a histogram.
   * \return          The histogram.
   */
  template <typename Label>
  static Histogram<Label> make_histogram(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

    Histogram<Label> histogram;
    for(typename std::vector<Example_CPtr>::const_iterator it = examples.begin(), iend = examples.end(); it != iend; ++it)
    {
      histogram.add((*it)->get_label());
    }

    return histogram;
  }

  /**
   * \brief Makes a probability mass function (PMF) from the label distribution of a set of examples.
   *
   * \param examples  The examples from whose label distribution we want to make a PMF.
   * \return          The PMF.
   */
  template <typename Label>
  static ProbabilityMassFunction<Label> make_pmf(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    return ProbabilityMassFunction<Label>(make_histogram(examples));
  }
  
  /**
   * \brief Creates a vector of examples with the same number of examples per class
   * 
   * \param labelSet                  The set of labels from which to draw the samples
   * \param numberOfSamplesPerClass   The number of samples to create per class
   * \return                          The set of generated examples
   */
  template <typename Label>
  static std::vector<boost::shared_ptr<const Example<Label> > > unit_circle_example_generator(const std::set<Label>& labelSet, size_t numberOfSamplesPerClass)
  {
    
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
    std::vector<Example_CPtr> exampleSet;

    const double pi = 3.14f;

    std::vector<Label> labelSetVector(labelSet.begin(), labelSet.end());

    //Defines the angle with which to rotate the unit vector [d1 d2]' 
    const float rotation_per_class = 2.f*pi/numberOfSamplesPerClass;
    
    //typename std::set<Label>::const_iterator set_iterator = labelSet.begin();
    const float d1 = 0.f;
    const float d2 = 1.f;
     
    for(size_t i = 0; i < numberOfSamplesPerClass; ++i)
    {
      for(int j = 0; j < labelSetVector.size(); ++j)
      {
        float angle = j*rotation_per_class;
        float r11 = cos(angle);
        float r12 = -sin(angle);
        float r21 = sin(angle);
        float r22 = cos(angle);
          
        //Rotation matrix (j*(2*pi/numberOfSamplesPerClass)); R = [cos(x) -sin(x); sin(x) cos(x)];
        float a = r11*d1 + r12*d2;
        float b = r21*d1 + r22*d2;
        
        Example_CPtr e(new Example<Label>(make_const_2d_descriptor(a,b), labelSetVector[j]));
        exampleSet.push_back( e );
      }
    }
    
    return exampleSet;
  }
};


  template <typename Label>
  class UnitCircleExampleGenerator
  {

  private:
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  private:
    float m_d1;
    float m_d2;

    //Defines the angle with which to rotate the unit vector [d1 d2]' 
    float m_rotationPerClass;
   
    std::map<Label,int> m_uniqueClassLabels;

    float m_pi;

  public:
    explicit UnitCircleExampleGenerator(std::map<Label,int> uniqueClassLabels)
    : m_d1(0.0f), m_d2(1.0f), m_pi(3.14f), m_uniqueClassLabels(uniqueClassLabels)
    {
      m_rotationPerClass = 2.0f*m_pi/m_uniqueClassLabels.size();
    }

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
