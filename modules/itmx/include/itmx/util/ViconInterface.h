/**
 * itmx: ViconInterface.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_VICONINTERFACE
#define H_ITMX_VICONINTERFACE

#include <map>
#include <set>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <vicon/Client.h>

#include <ORUtils/Math.h>

namespace itmx {

/**
 * \brief An instance of this class can be used to interface with a Vicon system.
 */
class ViconInterface
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The Vicon client. */
  mutable ViconDataStreamSDK::CPP::Client m_vicon;

  /** The relative transformation from world space to Vicon space (if known). */
  boost::optional<Matrix4f> m_worldToViconTransform;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon interface.
   *
   * \param host  The host on which the Vicon software is running (e.g. "<IP address>:<port>").
   */
  explicit ViconInterface(const std::string& host);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the Vicon interface.
   */
  ~ViconInterface();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  /** Deliberately private and unimplemented. */
  ViconInterface(const ViconInterface&);
  ViconInterface& operator=(const ViconInterface&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the current frame number.
   *
   * \return  The current frame number.
   */
  unsigned int get_frame_number() const;

  /**
   * \brief Gets the relative transformation from world space to Vicon space (if known).
   *
   * \return  The relative transformation from world space to Vicon space, if known, or boost::none otherwise.
   */
  const boost::optional<Matrix4f>& get_world_to_vicon_transform() const;

  /**
   * \brief Sets the relative transformation from world space to Vicon space.
   *
   * \param worldToViconTransform The relative transformation from world space to Vicon space.
   */
  void set_world_to_vicon_transform(const Matrix4f& worldToViconTransform);

  /**
   * \brief Attempts to get the positions of the markers for the Vicon subject with the specified name.
   *
   * This may fail if we move out of the range of the cameras or some of the markers are occluded.
   *
   * \param subjectName The name of the subject.
   * \return            The positions of the markers for the subject, indexed by name, or boost::none if they are temporarily unavailable.
   */
  boost::optional<std::map<std::string,Eigen::Vector3f> > try_get_marker_positions(const std::string& subjectName) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ViconInterface> ViconInterface_Ptr;
typedef boost::shared_ptr<const ViconInterface> ViconInterface_CPtr;

}

#endif
