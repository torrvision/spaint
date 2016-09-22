/**
 * spaint: FiducialDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIALDETECTOR
#define H_SPAINT_FIDUCIALDETECTOR

#include <map>

#include "Fiducial.h"
#include "../util/ITMObjectPtrTypes.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to detect fiducials in a 3D scene.
 */
class FiducialDetector
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the fiducial detector.
   */
  virtual ~FiducialDetector() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Detects fiducials in a view of a 3D scene.
   *
   * \param view        A view of the 3D scene.
   * \param pose        The pose from which the view was captured.
   * \param renderState A render state corresponding to the camera pose.
   * \return            The fiducials (if any) that have been detected in the view.
   */
  virtual std::map<std::string,Fiducial> detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose,
                                                          const VoxelRenderState_CPtr& renderState) const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const FiducialDetector> FiducialDetector_CPtr;

}

#endif
