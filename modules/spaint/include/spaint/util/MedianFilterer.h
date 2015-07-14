/**
 * spaint: MedianFilterer.h
 */

#ifndef H_SPAINT_MEDIANFILTERER
#define H_SPAINT_MEDIANFILTERER

#include "../imageprocessing/cuda/ImageProcessor_CUDA.h"
#include "../imageprocessing/interface/ImageProcessor.h"

namespace spaint {

/**
 * \brief TODO
 */
class MedianFilterer
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  unsigned int m_kernelWidth;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit MedianFilterer(unsigned int kernelWidth);

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief TODO
   */
  void operator()(const ITMUChar4Image_CPtr& input, const ITMUChar4Image_Ptr& output) const;
};

}

#endif
