/**
 * spaint: SLAMContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCONTEXT
#define H_SPAINT_SLAMCONTEXT

#include <boost/shared_ptr.hpp>

#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include "../util/SpaintVoxel.h"

namespace spaint {

/**
* \brief An instance of a class deriving from this one provides the shared context needed by a SLAM component.
*/
class SLAMContext
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the SLAM context.
   */
  virtual ~SLAMContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const View_Ptr& get_view() = 0;
  virtual VisualisationEngine_CPtr get_visualisation_engine() const = 0;
  virtual void set_view(ITMLib::ITMView *view) = 0;
};

}

#endif
