/**
 * itmx: RefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_REFININGRELOCALISER
#define H_ITMX_REFININGRELOCALISER

#include "Relocaliser.h"

namespace itmx {

/**
 * \brief An instance of a relocaliser class deriving from this one can be used to refine the results of another relocaliser.
 */
class RefiningRelocaliser : public Relocaliser
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The relocaliser whose results are being refined. */
  Relocaliser_Ptr m_innerRelocaliser;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a refining relocaliser.
   *
   * \param innerRelocaliser  The relocaliser whose results are being refined.
   */
  explicit RefiningRelocaliser(const Relocaliser_Ptr& innerRelocaliser);

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to determine the location from which an RGB-D image pair was acquired,
   *        thereby relocalising the camera with respect to the 3D scene.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   * \param initialPoses    A location in which to store the camera poses estimated by the inner relocaliser (if it succeeded),
   *                        or an empty vector otherwise.
   *
   * \return  The results of the relocalisation, from best to worst, or an empty vector otherwise.
   */
  virtual std::vector<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                         const Vector4f& depthIntrinsics, std::vector<ORUtils::SE3Pose>& initialPoses) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the relocaliser whose results are being refined.
   *
   * \return  The relocaliser whose results are being refined.
   */
  const Relocaliser_Ptr& get_inner_relocaliser();

  /**
   * \brief Gets the relocaliser whose results are being refined.
   *
   * \return  The relocaliser whose results are being refined.
   */
  Relocaliser_CPtr get_inner_relocaliser() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RefiningRelocaliser> RefiningRelocaliser_Ptr;
typedef boost::shared_ptr<const RefiningRelocaliser> RefiningRelocaliser_CPtr;

}

#endif
