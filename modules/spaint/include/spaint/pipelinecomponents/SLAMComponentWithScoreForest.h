/**
 * spaint: SLAMComponentWithScoreForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST
#define H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST

#include "SLAMComponent.h"

#include <boost/shared_ptr.hpp>

#include <DatasetRGBD7Scenes.hpp>
#include <DFBP.hpp>

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to perform simultaneous localisation and mapping (SLAM).
 */
class SLAMComponentWithScoreForest : public SLAMComponent
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a SLAM component performing relocalization using a Score Forest.
   *
   * \param context           The shared context needed for SLAM.
   * \param sceneID           The ID of the scene to reconstruct.
   * \param imageSourceEngine The engine used to provide input images to the fusion process.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode to use.
   * \param trackingMode      The tracking mode to use.
   */
  SLAMComponentWithScoreForest(const SLAMContext_Ptr& context, const std::string& sceneID, const ImageSourceEngine_Ptr& imageSourceEngine,
                TrackerType trackerType, const std::vector<std::string>& trackerParams, MappingMode mappingMode = MAP_VOXELS_ONLY,
                TrackingMode trackingMode = TRACK_VOXELS);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys a SLAM component.
   */
  virtual ~SLAMComponentWithScoreForest();

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  virtual TrackingResult process_relocalisation(TrackingResult trackingResult);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  cv::Mat build_rgbd_image(const ITMUChar4Image_Ptr &rgb, const ITMShortImage_Ptr &depth) const;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  boost::shared_ptr<DatasetRGBD7Scenes> m_dataset;
  boost::shared_ptr<DFBP> m_forest;

};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMComponentWithScoreForest> SLAMComponentWithScoreForest_Ptr;

}

#endif
