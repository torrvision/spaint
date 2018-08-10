/**
 * orx: Screw.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ORX_SCREW
#define H_ORX_SCREW

#include <ORUtils/Vector.h>

namespace orx {

/**
 * \brief An instance of an instantiation of this class template represents a transformation in screw form.
 *
 *        See "Dual-Quaternions: From Classical Mechanics to Computer Graphics and Beyond" by Ben Kenwright.
 */
template <typename T>
struct Screw
{
  //#################### PUBLIC VARIABLES ####################

  /** The screw angle. */
  T angle;

  /** The screw direction. */
  ORUtils::Vector3<T> direction;

  /** The screw moment. */
  ORUtils::Vector3<T> moment;

  /** The screw pitch. */
  T pitch;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a screw transformation.
   *
   * \param angle_      The screw angle.
   * \param pitch_      The screw pitch.
   * \param direction_  The screw direction.
   * \param moment_     The screw moment.
   */
  Screw(T angle_, T pitch_, const ORUtils::Vector3<T>& direction_, const ORUtils::Vector3<T>& moment_)
  : angle(angle_), direction(direction_), moment(moment_), pitch(pitch_)
  {}
};

//#################### TYPEDEFS ####################

typedef Screw<double> Screwd;
typedef Screw<float> Screwf;

}

#endif
