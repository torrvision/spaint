/**
 * tvgutil: AttitudeUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_ATTITUDEUTIL
#define H_TVGUTIL_ATTITUDEUTIL

#include <vector>
#include <cstdlib>
#include <cmath>
#include <stdexcept>

namespace tvgutil {

/**
 * \brief This struct may be used to convert between various 3D attitude representations.
 */
struct AttitudeUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the L2 norm of a vector.
   *
   * \param v  The vector.
   * \return   The L2 norm of the vector.
   */
  template <typename T>
  static T l2_norm(const T *v, size_t elementCount)
  {
    T sumSquares = 0.0;
    for(size_t i = 0; i < elementCount; ++i)
    {
      sumSquares += v[i] * v[i];
    }
    return sqrt(sumSquares);
  }

  template <typename T>
  static T l2_norm(const std::vector<T>& v)
  {
    return l2_norm(&v.front(), v.size());
  }

  /**
   * \brief Converts a rotation matrix to a quaternion.
   *
   * \param matrix   The rotation matrix in row-major format.
   * \param q        The quaternion.
   */
  template <typename T>
  static void rotation_matrix_to_quaternion(const T *matrix, T *q)
  {
    /* Taken from "James Diebel. Representing Attitude: Euler Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford University, Palo Alto, CA." */

    int variant = 0;
    if((matrix[4]>-matrix[8])&&(matrix[0]>-matrix[4])&&(matrix[0]>-matrix[8]))
    {
      variant=0;
    }
    else if((matrix[4]<-matrix[8])&&(matrix[0]>matrix[4])&&(matrix[0]> matrix[8]))
    {
      variant=1;
    } else if((matrix[4]> matrix[8])&&(matrix[0]<matrix[4])&&(matrix[0]<-matrix[8]))
    {
      variant=2;
    } else if((matrix[4]<matrix[8])&&(matrix[0]<-matrix[4])&&(matrix[0]< matrix[8]))
    {
      variant=3;
    }

    // choose the numerically best variant...
    double denom = 1.0;
    if (variant==0)
    {
      denom+=matrix[0]+matrix[4]+matrix[8];
    } else
    {
      int tmp = variant*4;
      denom+=matrix[tmp-4];
      denom-=matrix[tmp%12];
      denom-=matrix[(tmp+4)%12];
    }
    denom=sqrt(denom);
    q[variant] = 0.5*denom;

    denom *= 2.0;
    switch(variant) {
    case 0:
            q[1] = (matrix[5] - matrix[7])/denom;
            q[2] = (matrix[6] - matrix[2])/denom;
            q[3] = (matrix[1] - matrix[3])/denom;
            break;
    case 1:
            q[0] = (matrix[5] - matrix[7])/denom;
            q[2] = (matrix[1] + matrix[3])/denom;
            q[3] = (matrix[6] + matrix[2])/denom;
            break;
    case 2:
            q[0] = (matrix[6] - matrix[2])/denom;
            q[1] = (matrix[1] + matrix[3])/denom;
            q[3] = (matrix[5] + matrix[7])/denom;
            break;
    case 3:
            q[0] = (matrix[1] - matrix[3])/denom;
            q[1] = (matrix[6] + matrix[2])/denom;
            q[2] = (matrix[5] + matrix[7])/denom;
            break;
    }
    //q[0]*=-1.0; // TODO: I think this is a bug.
  }

  /**
   * \brief Converts a quaternion to a rotation matrix.
   *
   * \param q      The quaternion, where q[0] is the real part and q[1-3] are the imaginary parts.
   * \param matrix The rotation matrix in row-major format.
   */
  template <typename T>
  static void quaternion_to_rotation_matrix(const T *q, T *matrix)
  {
          matrix[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
          matrix[4] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
          matrix[8] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
          matrix[3] = 2*(q[1]*q[2] - q[0]*q[3]);
          matrix[1] = 2*(q[1]*q[2] + q[0]*q[3]);
          matrix[6] = 2*(q[1]*q[3] + q[0]*q[2]);
          matrix[2] = 2*(q[1]*q[3] - q[0]*q[2]);
          matrix[7] = 2*(q[2]*q[3] - q[0]*q[1]);
          matrix[5] = 2*(q[2]*q[3] + q[0]*q[1]);
  }

  /**
   * \brief Converts a rotation vector to a quaternion.
   *
   * \param r  The rotation vector.
   * \return   The equivalent quaternion.
   */
  template <typename T>
  static void rotation_vector_to_quaternion(const T *rv, T *q)
  {
    float axis[3];
    float angle;
    rotation_vector_to_axis_angle(rv, axis, &angle);

    // Create the quaternion.
    T sinHalfTheta = sin(angle/2.0f);
    q[0] = cos(angle/2.0f);
    q[1] = axis[0] * sinHalfTheta;
    q[2] = axis[1] * sinHalfTheta;
    q[3] = axis[2] * sinHalfTheta;
  }

  template <typename T>
  static void quaternion_to_rotation_vector(const T *q, T *rv)
  {
    T axis[3];
    T angle;
    quaternion_to_axis_angle(q, axis, &angle);
    axis_angle_to_rotation_vector(axis, &angle, rv);
  }

  template <typename T>
  static void axis_angle_to_rotation_vector(const T *axis, const T *angle, T *rv)
  {
    rv[0] = axis[0] * *angle;
    rv[1] = axis[1] * *angle;
    rv[2] = axis[2] * *angle;
  }

  template <typename T>
  static void quaternion_to_axis_angle(const T *q, T *axis, T *angle)
  {
    *angle = 2.0f * acos(q[0]);

    // If the real part is one, a zero will appear in the denominator of the multiplier.
    T realSquared = q[0] * q[0];
    if(realSquared < 1.0f)
    {
      T multiplier = 1.0f / sqrt(1.0f - realSquared);
      axis[0] = q[1] * multiplier;
      axis[1] = q[2] * multiplier;
      axis[2] = q[3] * multiplier;
    }
    else if(realSquared == 1.0f)
    {
      *angle = 0.0f;
      axis[0] = 1.0f;
      axis[1] = 0.0f;
      axis[2] = 0.0f;
    }
    else
    {
      throw std::runtime_error("Input not a unit quaternion");
    }
  }

  template <typename T>
  static void rotation_matrix_to_axis_angle(const T *matrix, T *axis, T *angle)
  {
    T q[4];
    rotation_matrix_to_quaternion(matrix, q);
    quaternion_to_axis_angle(q, axis, angle);
  }

  template <typename T>
  static void rotation_vector_to_axis_angle(const T *rv, T *axis, T *angle)
  {
    const T minval = 1e-20;
    size_t elementCount = 3;
    T rTheta = l2_norm(rv, elementCount);

    // Clip the magnitude to a minimum value to prevent division by zero.
    if(rTheta < minval) rTheta = minval;
    *angle = rTheta;

    std::vector<T> rUnit(elementCount);
    for(size_t i = 0; i < elementCount; ++i)
    {
      axis[i] = rv[i] / rTheta;
    }
  }

  template <typename T>
  static void axis_angle_to_rotation_matrix(const T *axis, const T *angle, T *matrix)
  {
    float rv[3];
    axis_angle_to_rotation_vector(axis, angle, rv);

    float q[4];
    rotation_vector_to_quaternion(rv, q);

    quaternion_to_rotation_matrix(q, matrix);
  }
};

}

#endif
