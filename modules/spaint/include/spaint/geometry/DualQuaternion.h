/**
 * spaint: DualQuaternion.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_DUALQUATERNION
#define H_SPAINT_DUALQUATERNION

#include <stdexcept>

#include <boost/mpl/identity.hpp>

#include <ORUtils/MathUtils.h>

#include <tvgutil/misc/AttitudeUtil.h>

#include "DualNumber.h"
#include "Screw.h"

namespace spaint {

/**
 * \brief An instance of an instantiation of this class template represents a dual quaternion,
 *        an extension of a normal quaternion that can represent a full rigid-body transform.
 *
 *        See "Dual Quaternions for Rigid Transformation Blending" by Kavan et al.
 */
template <typename T>
class DualQuaternion
{
  //#################### PUBLIC VARIABLES ####################
public:
  /** The x^, y^, z^ and w^ components of the dual quaternion q^, such that q^ = w^ + x^.i + y^.j + z^.k. */
  DualNumber<T> x, y, z, w;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a dual quaternion whose components are all zero.
   */
  DualQuaternion() {}

  /**
   * \brief Constructs a dual quaternion with the specified components.
   *
   * \param w_  The w^ component.
   * \param x_  The x^ component.
   * \param y_  The y^ component.
   * \param z_  The z^ component.
   */
  DualQuaternion(const DualNumber<T>& w_, const DualNumber<T>& x_, const DualNumber<T>& y_, const DualNumber<T>& z_)
  : x(x_), y(y_), z(z_), w(w_)
  {}

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the angle needed to rotate one dual quaternion representing a pure rotation into another.
   *
   * This can function as a distance metric between two pure rotations.
   *
   * \param q1  The first dual quaternion.
   * \param q2  The second dual quaternion.
   * \return    The angle needed to rotate one dual quaternion into the other.
   */
  static T angle_between_rotations(const DualQuaternion<T>& q1, const DualQuaternion<T>& q2)
  {
    DualQuaternion<T> q1toq2 = (q2 * q1.conjugate()).normalised();
    return length(q1toq2.get_rotation());
  }

  /**
   * \brief Checks whether two dual quaternions are approximately equal, up to a tolerance.
   *
   * \param lhs       The first dual quaternion.
   * \param rhs       The second dual quaternion.
   * \param tolerance The tolerance value.
   * \return          true, if the two dual quaternions are approximately equal, or false otherwise.
   */
  static bool close(const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs, T tolerance = 1e-4f)
  {
    return DualNumber<T>::close(lhs.w, rhs.w, tolerance) &&
           DualNumber<T>::close(lhs.x, rhs.x, tolerance) &&
           DualNumber<T>::close(lhs.y, rhs.y, tolerance) &&
           DualNumber<T>::close(lhs.z, rhs.z, tolerance);
  }

  /**
   * \brief Constructs a dual quaternion that represents a 3D point.
   *
   * \param p The point.
   * \return  The dual quaternion.
   */
  static DualQuaternion<T> from_point(const ORUtils::Vector3<T>& p)
  {
    return DualQuaternion<T>(
      DualNumber<T>(1, 0),
      DualNumber<T>(0, p.x),
      DualNumber<T>(0, p.y),
      DualNumber<T>(0, p.z)
    );
  }

  /**
   * \brief Constructs a dual quaternion that represents a rotation of a particular angle about an axis.
   *
   * \param axis                The rotation axis.
   * \param angle               The rotation angle.
   * \return                    The dual quaternion.
   * \throws std::runtime_error If the rotation axis is invalid.
   */
  template <typename U>
  static DualQuaternion<T> from_rotation(ORUtils::Vector3<U> axis, typename boost::mpl::identity<U>::type angle)
  {
    U axisLengthSquared = dot(axis, axis);
    if(fabs(axisLengthSquared - 1) > 1e-9)
    {
      if(axisLengthSquared > 1e-6)
      {
        // FIXME: Currently, the implementation of Vector3<T>::normalised always returns a Vector3f,
        //        which is unhelpful when trying to normalise a Vector3d. This should be fixed.
        U axisLength = sqrt(axisLengthSquared);
        axis.x /= axisLength;
        axis.y /= axisLength;
        axis.z /= axisLength;
      }
      else throw std::runtime_error("Error: Could not construct dual quaternion - bad rotation axis");
    }

    U cosHalfTheta = cos(angle/2);
    U sinHalfTheta = sqrt(1 - cosHalfTheta*cosHalfTheta);
    return DualQuaternion<T>(cosHalfTheta, sinHalfTheta * axis.x, sinHalfTheta * axis.y, sinHalfTheta * axis.z);
  }

  /**
   * \brief Constructs a dual quaternion that corresponds to a rotation expressed as a Lie rotation vector.
   *
   * \note A Lie rotation vector v encodes a rotation of |v| about the axis v / |v|.
   *
   * \param rot The Lie rotation vector.
   * \return    The dual quaternion.
   */
  template <typename U>
  static DualQuaternion<T> from_rotation(const ORUtils::Vector3<U>& rot)
  {
    U lengthSquared = dot(rot, rot);
    if(lengthSquared > 1e-6)
    {
      U length = sqrt(lengthSquared);
      return from_rotation(rot / length, length);
    }
    else return identity();
  }

  /**
   * \brief Constructs a dual quaternion that corresponds to a transformation expressed in screw form.
   *
   * \param screw The screw transformation.
   * \return      The dual quaternion.
   */
  static DualQuaternion<T> from_screw(const Screw<T>& screw)
  {
    // See "Dual-Quaternions: From Classical Mechanics to Computer Graphics and Beyond" by Ben Kenwright.
    T c = cos(screw.angle / 2), s = sin(screw.angle / 2);
    return DualQuaternion<T>(
      DualNumber<T>(c, -screw.pitch * s / 2),
      DualNumber<T>(screw.direction.x * s, screw.moment.x * s + screw.pitch * screw.direction.x * c / 2),
      DualNumber<T>(screw.direction.y * s, screw.moment.y * s + screw.pitch * screw.direction.y * c / 2),
      DualNumber<T>(screw.direction.z * s, screw.moment.z * s + screw.pitch * screw.direction.z * c / 2)
    );
  }

  /**
   * \brief Constructs a dual quaternion that represents a translation by a 3D vector.
   *
   * \param t The translation vector.
   * \return  The dual quaternion.
   */
  static DualQuaternion<T> from_translation(const ORUtils::Vector3<T>& t)
  {
    return DualQuaternion<T>(
      DualNumber<T>(1, 0),
      DualNumber<T>(0, static_cast<T>(t.x / 2.0)),
      DualNumber<T>(0, static_cast<T>(t.y / 2.0)),
      DualNumber<T>(0, static_cast<T>(t.z / 2.0))
    );
  }

  /**
   * \brief Makes a dual quaternion that corresponds to the identity matrix.
   *
   * \return  A dual quaternion that corresponds to the identity matrix.
   */
  static DualQuaternion<T> identity()
  {
    return DualQuaternion<T>(1, 0, 0, 0);
  }

  /**
   * \brief Performs a weighted linear blend of the specified set of unit dual quaternions.
   *
   * The result is normalised to ensure that it is another unit dual quaternion.
   *
   * \param dqs     The input dual quaternions.
   * \param weights The corresponding weights.
   * \param count   The number of dual quaternions being blended.
   * \return        The result of blending the dual quaternions.
   */
  static DualQuaternion<T> linear_blend(const DualQuaternion<T> *dqs, const T *weights, int count)
  {
    DualQuaternion<T> result;
    for(int i = 0; i < count; ++i)
    {
      result += weights[i] * dqs[i];
    }
    return result.normalised();
  }

  /**
   * \brief Interpolates between two dual quaternions using the ScLERP approach.
   *
   * \param lhs The first dual quaternion.
   * \param rhs The second dual quaternion.
   * \param t   The interpolation parameter (in the range [0,1]).
   */
  static DualQuaternion<T> sclerp(const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs, T t)
  {
    return lhs * (lhs.conjugate() * rhs).pow(t);
  }

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Adds another dual quaternion to this one.
   *
   * \param rhs The other dual quaternion.
   * \return    This dual quaternion.
   */
  DualQuaternion<T>& operator+=(const DualQuaternion<T>& rhs)
  {
    w += rhs.w; x += rhs.x; y += rhs.y; z += rhs.z;
    return *this;
  }

  /**
   * \brief Multiplies this dual quaternion by another one.
   *
   * \param rhs The other dual quaternion.
   * \return    This dual quaternion.
   */
  DualQuaternion<T>& operator*=(const DualQuaternion<T>& rhs)
  {
    *this = *this * rhs;
    return *this;
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Applies the transformation represented by this dual quaternion to a 3D point.
   *
   * \param p The 3D point.
   * \return  The transformed point.
   */
  ORUtils::Vector3<T> apply(const ORUtils::Vector3<T>& p) const
  {
    DualQuaternion<T> result(*this);
    result *= from_point(p);
    result *= dual_conjugate();
    return result.to_point();
  }

  /**
   * \brief Calculates the conjugate of this dual quaternion.
   *
   * \return  The conjugate of this dual quaternion.
   */
  DualQuaternion<T> conjugate() const
  {
    return DualQuaternion<T>(w, -x, -y, -z);
  }

  /**
   * \brief Calculates the "dual conjugate" of this dual quaternion.
   *
   * This involves applying both quaternion and dual conjugation.
   *
   * \return  The "dual conjugate" of this dual quaternion.
   */
  DualQuaternion<T> dual_conjugate() const
  {
    return DualQuaternion<T>(w.conjugate(), -x.conjugate(), -y.conjugate(), -z.conjugate());
  }

  /**
   * \brief Gets a Lie vector corresponding to the rotation component of the rigid-body transform represented by this dual quaternion.
   *
   * \return  A Lie vector corresponding to the rotation component of the rigid-body transform represented by this dual quaternion.
   */
  ORUtils::Vector3<T> get_rotation() const
  {
    T q[] = { w.r, x.r, y.r, z.r };
    T rv[3];
    tvgutil::AttitudeUtil::quaternion_to_rotation_vector(q, rv);
    return ORUtils::Vector3<T>(rv[0], rv[1], rv[2]);
  }

  /**
   * \brief Gets a dual quaternion corresponding to the rotation component of the rigid-body transform represented by this dual quaternion.
   *
   * \return  Gets a dual quaternion corresponding to the rotation component of the rigid-body transform represented by this dual quaternion.
   */
  DualQuaternion<T> get_rotation_part() const
  {
    return DualQuaternion<T>(w.r, x.r, y.r, z.r);
  }

  /**
   * \brief Gets the translation component of the rigid-body transform represented by this dual quaternion.
   *
   * \return  The translation component of the rigid-body transform represented by this dual quaternion.
   */
  ORUtils::Vector3<T> get_translation() const
  {
    DualQuaternion<T> tp = get_translation_part();
    return static_cast<T>(2) * ORUtils::Vector3<T>(tp.x.d, tp.y.d, tp.z.d);
  }

  /**
   * \brief Gets a dual quaternion corresponding to the translation component of the rigid-body transform represented by this dual quaternion.
   *
   * \return  Gets a dual quaternion corresponding to the translation component of the rigid-body transform represented by this dual quaternion.
   */
  DualQuaternion<T> get_translation_part() const
  {
    return *this * get_rotation_part().conjugate();
  }

  /**
   * \brief Calculates the norm of this dual quaternion.
   *
   * \return  The norm of this dual quaternion.
   */
  DualNumber<T> norm() const
  {
    return (w * w + x * x + y * y + z * z).sqrt();
  }

  /**
   * \brief Calculates a normalised version of this dual quaternion.
   *
   * \return  A normalised version of this dual quaternion.
   */
  DualQuaternion<T> normalised() const
  {
    DualNumber<T> invNorm = norm().inverse();
    return invNorm * *this;
  }

  /**
   * \brief Calculates an exponent of this dual quaternion.
   *
   * \param exponent  The exponent.
   * \return          The result of raising this dual quaternion to the specified power.
   */
  DualQuaternion<T> pow(T exponent) const
  {
    Screw<T> s = to_screw();
    s.angle *= exponent;
    s.pitch *= exponent;
    return from_screw(s);
  }

  /**
   * \brief Calculates a screw representation of this dual quaternion.
   *
   * \return  A screw representation of this dual quaternion.
   */
  Screw<T> to_screw() const
  {
    // See "Dual-Quaternions: From Classical Mechanics to Computer Graphics and Beyond" by Ben Kenwright.
    ORUtils::Vector3<T> vr(x.r, y.r, z.r);
    ORUtils::Vector3<T> vd(x.d, y.d, z.d);
    const T invVrLen = 1 / length(vr);
    const T& wr = w.r;
    const T& wd = w.d;

    T angle = 2 * acos(CLAMP(wr, static_cast<T>(-1), static_cast<T>(1)));
    T pitch = -2 * wd * invVrLen;
    ORUtils::Vector3<T> direction = vr * invVrLen;
    ORUtils::Vector3<T> moment = (vd - direction * (pitch * wr / 2)) * invVrLen;

    return Screw<T>(angle, pitch, direction, moment);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Converts this dual quaternion to a 3D point (assuming that it represents one in the first place).
   *
   * \return  A 3D point corresponding to this dual quaternion.
   */
  ORUtils::Vector3<T> to_point() const
  {
    return ORUtils::Vector3<T>(x.d, y.d, z.d);
  }
};

//#################### NON-MEMBER OPERATORS ####################

/**
 * \brief Scales a dual quaternion by the specified factor.
 *
 * \param factor  The scaling factor.
 * \param q       The dual quaternion.
 * \return        A scaled version of the dual quaternion.
 */
template <typename T>
DualQuaternion<T> operator*(const DualNumber<T>& factor, const DualQuaternion<T>& q)
{
  return DualQuaternion<T>(factor * q.w, factor * q.x, factor * q.y, factor * q.z);
}

/**
 * \brief Scales a dual quaternion by the specified factor.
 *
 * \param factor  The scaling factor.
 * \param q       The dual quaternion.
 * \return        A scaled version of the dual quaternion.
 */
template <typename T>
DualQuaternion<T> operator*(T factor, const DualQuaternion<T>& q)
{
  return DualNumber<T>(factor) * q;
}

/**
 * \brief Multiplies two dual quaternions together.
 *
 * \param q1  The first operand.
 * \param q2  The second operand.
 * \return    The result of the operation.
 */
template <typename T>
DualQuaternion<T> operator*(const DualQuaternion<T>& q1, const DualQuaternion<T>& q2)
{
  /*
  Note that it's possible to optimise this if necessary.

  See: https://github.com/sgolodetz/hesperus2/blob/master/source/engine/core/hesp/math/quaternions/Quaternion.cpp
  */
  return DualQuaternion<T>(
    q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
    q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
    q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
    q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
  );
}

//#################### STREAM OPERATORS ####################

template <typename T>
std::ostream& operator<<(std::ostream& os, const DualQuaternion<T>& rhs)
{
  os << '[' << rhs.w << ',' << rhs.x << ',' << rhs.y << ',' << rhs.z << ']';
  return os;
}

//#################### TYPEDEFS ####################

typedef DualQuaternion<double> DualQuatd;
typedef DualQuaternion<float> DualQuatf;

}

#endif
