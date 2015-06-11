/**
 * spaint: RiftTracker.h
 */

#ifndef H_SPAINT_RIFTTRACKER
#define H_SPAINT_RIFTTRACKER

#include <ITMLib/Engine/ITMTracker.h>

#include <OVR_CAPI.h>

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
  virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

  /** Override */
  virtual void UpdateInitialPose(ITMTrackingState *trackingState);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Extracts a rotation matrix suitable for use in an InfiniTAM pose from the Rift's tracking state.
   *
   * \param riftTrackingState The Rift's tracking state.
   * \return                  The rotation matrix corresponding to the Rift's tracking state.
   */
  static Matrix3f extract_rotation_matrix(const ovrTrackingState& riftTrackingState);

  /**
   * \brief Attempts to update the InfiniTAM tracking state using information from the Rift.
   *
   * The update can fail if it wasn't possible to read information from the Rift.
   *
   * \param trackingState The InfiniTAM tracking state.
   * \return              true, if the update succeeded, or false otherwise.
   */
  bool try_update_tracking_state(ITMTrackingState *trackingState) const;
};

}

#endif
