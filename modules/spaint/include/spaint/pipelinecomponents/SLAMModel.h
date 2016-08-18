/**
 * spaint: SLAMModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMMODEL
#define H_SPAINT_SLAMMODEL

#include <boost/shared_ptr.hpp>

#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include "../util/SpaintVoxel.h"

namespace spaint {

/**
* \brief TODO
*/
class SLAMModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief TODO
   */
  virtual ~SLAMModel() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const View_Ptr& get_view() const = 0;

  /**
   * \brief TODO
   */
  virtual const VisualisationEngine_Ptr& get_visualisation_engine() const = 0;

  /**
   * \brief TODO
   */
  virtual void set_view(ITMLib::ITMView *view) = 0;
};

}

#endif
