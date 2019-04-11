/**
 * spaint: RelocaliserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#ifndef H_SPAINT_RELOCALISERFACTORY
#define H_SPAINT_RELOCALISERFACTORY

#include <itmx/base/ITMObjectPtrTypes.h>

#include <orx/relocalisation/Relocaliser.h>

namespace spaint {

/**
 * \brief This class can be used to construct relocalisers.
 */
class RelocaliserFactory
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::function<std::vector<ORUtils::SE3Pose>()> GroundTruthProvider;
  typedef boost::function<orx::Relocaliser_Ptr(const orx::Relocaliser_Ptr&)> Refiner;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a relocaliser of the specified type.
   *
   * \param relocaliserType           The type of relocaliser to construct.
   * \param depthImageSize            The size of the depth input images.
   * \param relocaliseEveryFrame      Whether or not we're relocalising and training after processing every frame, for evaluation purposes.
   * \param refineWithICP             A callback function that can be used to decorate a relocaliser with one that performs ICP refinement.
   * \param getGroundTruthTrajectory  A callback function that can be used to get the ground truth test trajectory.
   * \param settings                  The InfiniTAM settings.
   */
  static orx::Relocaliser_Ptr make_relocaliser(std::string relocaliserType, const Vector2i& depthImageSize, bool relocaliseEveryFrame,
                                               const Refiner& refineWithICP, const GroundTruthProvider& getGroundTruthTrajectory,
                                               const Settings_CPtr& settings);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a "simple" relocaliser (i.e. a relocaliser that is not a composite) of the specified type.
   *
   * \param relocaliserType           The type of relocaliser to construct.
   * \param relocaliserNamespace      The namespace associated with the settings that are specific to the relocaliser.
   * \param depthImageSize            The size of the depth input images.
   * \param relocaliseEveryFrame      Whether or not we're relocalising and training after processing every frame, for evaluation purposes.
   * \param refineWithICP             A callback function that can be used to decorate a relocaliser with one that performs ICP refinement.
   * \param getGroundTruthTrajectory  A callback function that can be used to get the ground truth test trajectory.
   * \param settings                  The InfiniTAM settings.
   */
  static orx::Relocaliser_Ptr make_simple_relocaliser(const std::string& relocaliserType, const std::string& relocaliserNamespace, const Vector2i& depthImageSize,
                                                      bool relocaliseEveryFrame, const Refiner& refineWithICP, const GroundTruthProvider& getGroundTruthTrajectory,
                                                      const Settings_CPtr& settings);
};

}

#endif
