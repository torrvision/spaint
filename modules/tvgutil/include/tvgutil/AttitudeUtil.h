/**
 * tvgutil: AttitudeUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_ATTITUDEUTIL
#define H_TVGUTIL_ATTITUDEUTIL

#include <vector>
#include <cstdlib>
#include <cmath>

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
    static const T minval = 1e-20;
    size_t elementCount = 3;

    // First extract the axis-angle representation.
    T rTheta = l2_norm(rv, elementCount);

    // Clip the magnitude to a minimum value to prevent division by zero.
    if(rTheta < minval) rTheta = minval;

    std::vector<T> rUnit(elementCount);
    for(size_t i = 0; i < elementCount; ++i)
    {
      rUnit[i] = rv[i] / rTheta;
    }

    // Create the quaternion.
    T sinHalfTheta = sin(rTheta/2.0f);
    q[0] = cos(rTheta/2.0f);
    q[1] = rUnit[0] * sinHalfTheta;
    q[2] = rUnit[1] * sinHalfTheta;
    q[3] = rUnit[2] * sinHalfTheta;
  }

  template <typename T>
  static void quaternion_to_rotation_vector(const T *q, T *rv)
  {
    T multiplier = (2.0f * acos(q[0]))/sqrt(1.0f - q[0]*q[0]);
    rv[0] = multiplier * q[1];
    rv[1] = multiplier * q[2];
    rv[2] = multiplier * q[3];
  }
};

}

#endif
