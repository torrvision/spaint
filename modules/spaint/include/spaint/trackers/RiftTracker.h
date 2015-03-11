/**
 * spaint: RiftTracker.h
 */

#ifndef H_SPAINT_RIFTTRACKER
#define H_SPAINT_RIFTTRACKER

#include <ITMLib/Engine/ITMTracker.h>

#include <OVR.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to track the rotation of the camera using an Oculus Rift.
 */
class RiftTracker : public ITMLib::Engine::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The Rift handle. */
  ovrHmd m_hmd;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Rift tracker.
   */
  RiftTracker();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void SetInitialPose(ITMTrackingState *trackingState);

  /** Override */
  virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Extracts a rotation matrix suitable for use in an InfiniTAM pose from the Rift's tracking state.
   *
   * \param riftTrackingState The Rift's tracking state.
   * \return                  The rotation matrix corresponding to the Rift's tracking state.
   */
  static Matrix3f extract_rotation_matrix(const ovrTrackingState& riftTrackingState);
};

}

#endif
