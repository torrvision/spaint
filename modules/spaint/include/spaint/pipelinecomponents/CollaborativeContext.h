/**
 * spaint: CollaborativeContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECONTEXT
#define H_SPAINT_COLLABORATIVECONTEXT

#include <itmx/relocalisation/RefiningRelocaliser.h>

#include "../slamstate/SLAMState.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a collaborative component.
 */
class CollaborativeContext
{
  //#################### PRIVATE VARIABLES ####################
private:
  /**
   * Accumulated samples of the relative transformations between the different submaps. Each sample for (scene i, scene j)
   * expresses an estimate of the transformation from the coordinate system of scene j to that of scene i.
   */
  std::map<std::pair<std::string,std::string>,std::vector<ORUtils::SE3Pose> > m_relativeTransformSamples;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the collaborative context.
   */
  virtual ~CollaborativeContext();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual itmx::RefiningRelocaliser_CPtr get_relocaliser(const std::string& sceneID) const = 0;
  virtual std::vector<std::string> get_scene_ids() const = 0;
  //virtual const Settings_CPtr& get_settings() const = 0;
  virtual SLAMState_CPtr get_slam_state(const std::string& sceneID) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to get an estimate of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \return        An estimate of the transformation from the coordinate system of scene j to that of scene i,
   *                if possible, or boost::none otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose> try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Adds a sample of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \param sample  A sample of the transformation from the coordinate system of scene j to that of scene i.
   */
  void add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const ORUtils::SE3Pose& sample);

  //#################### FRIENDS ####################

  friend class CollaborativeComponent;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeContext> CollaborativeContext_Ptr;

}

#endif
