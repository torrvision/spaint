/**
 * tvgutil: AttitudeUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_ATTITUDEUTIL
#define H_TVGUTIL_ATTITUDEUTIL

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace tvgutil {

/**
 * \brief This class may be used to convert between various 3D attitude representations.
 *
 * Note: The conversions were taken from "James Diebel. Representing Attitude: Euler Angles,
 * Quaternions, and Rotation Vectors. Technical Report, Stanford University, Palo Alto, CA."
 */
class AttitudeUtil
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the two standard orders in which a rotation matrix can be laid out in a linear array.
   */
  enum Order
  {
    COL_MAJOR,
    ROW_MAJOR
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Converts from an axis-angle representation to a rotation matrix.
   *
   * \param axis   The normalised axis of rotation.
   * \param angle  The angle of rotation (in radians).
   * \param matrix The rotation matrix.
   * \param order  The order in which the elements of the rotation matrix are linearised.
   */
  template <typename T>
  static void axis_angle_to_rotation_matrix(const T *axis, const T *angle, T *matrix, Order order)
  {
    T rv[3];
    axis_angle_to_rotation_vector(axis, angle, rv);

    T q[4];
    rotation_vector_to_quaternion(rv, q);

    quaternion_to_rotation_matrix(q, matrix, order);
  }

  /**
   * \brief Converts from an axis-angle representation to a rotation vector.
   *
   * \param axis  The normalised axis of rotation.
   * \param angle The angle of rotation (in radians).
   * \param rv    The rotation vector.
   */
  template <typename T>
  static void axis_angle_to_rotation_vector(const T *axis, const T *angle, T *rv)
  {
    rv[0] = axis[0] * *angle;
    rv[1] = axis[1] * *angle;
    rv[2] = axis[2] * *angle;
  }

  /**
   * \brief Converts from a unit quaternion to an axis-angle representation.
   *
   * \param q       The unit quaternion.
   * \param axis    The normalised axis of rotation.
   * \param angle   The angle of the rotation.
   */
  template <typename T>
  static void quaternion_to_axis_angle(const T *q, T *axis, T *angle)
  {
    const T TOL = 1e-4f;
    T realSquared = q[0] * q[0];

    // A special case is required when the real part of the unit quaternion is one:
    // q = [1 0 0 0] (the identity quaternion), to avoid a division by zero.
    // Note: since the quaternion is normalised to unit length and the real part is one,
    // the imaginary parts must be zero.
    // The identity quaternion corresponds to a zero rotation around an arbitrary axis,
    // and here we return a zero rotation around the x-axis.
    if(realSquared < 1.0f - TOL)
    {
      *angle = 2.0f * acos(q[0]);
      T multiplier = 1.0f / sqrt(1.0f - realSquared);
      axis[0] = q[1] * multiplier;
      axis[1] = q[2] * multiplier;
      axis[2] = q[3] * multiplier;
    }
    else if(fabs(realSquared - 1.0f) <= TOL)
    {
      *angle = 0.0f;
      axis[0] = 1.0f;
      axis[1] = 0.0f;
      axis[2] = 0.0f;
    }
    else throw std::runtime_error("Input not a unit quaternion");
  }

  /**
   * \brief Converts a unit quaternion to a rotation matrix.
   *
   * \param q      The unit quaternion.
   * \param matrix The rotation matrix.
   * \param order  The order in which the elements of the rotation matrix are linearised.
   */
  template <typename T>
  static void quaternion_to_rotation_matrix(const T *q, T *matrix, Order order)
  {
    quaternion_to_row_major_rotation_matrix(q, matrix);
    if(order == COL_MAJOR) transpose_matrix3_in_place(matrix);
  }

  /**
   * \brief Converts a unit quaternion to a rotation vector.
   *
   * \param q  The unit quaternion.
   * \param rv The rotation vector.
   */
  template <typename T>
  static void quaternion_to_rotation_vector(const T *q, T *rv)
  {
    T axis[3];
    T angle;
    quaternion_to_axis_angle(q, axis, &angle);
    axis_angle_to_rotation_vector(axis, &angle, rv);
  }

  /**
   * \brief Converts a rotation matrix to an axis-angle representation.
   *
   * \param matrix  The rotation matrix.
   * \param axis    The normalised axis of rotation.
   * \param angle   The angle of rotation (in radians).
   * \param order   The order in which the elements of the rotation matrix are linearised.
   */
  template <typename T>
  static void rotation_matrix_to_axis_angle(const T *matrix, T *axis, T *angle, Order order)
  {
    T q[4];
    rotation_matrix_to_quaternion(matrix, q, order);
    quaternion_to_axis_angle(q, axis, angle);
  }

  /**
   * \brief Converts a rotation matrix to a unit quaternion.
   *
   * \param matrix  The rotation matrix.
   * \param q       The unit quaternion.
   * \param order   The order in which the elements of the rotation matrix are linearised.
   */
  template <typename T>
  static void rotation_matrix_to_quaternion(const T *matrix, T *q, Order order)
  {
    const T *rowMajorMatrix = matrix;
    T temp[9];
    if(order == COL_MAJOR)
    {
      transpose_matrix3(matrix, temp);
      rowMajorMatrix = temp;
    }
    row_major_rotation_matrix_to_quaternion(rowMajorMatrix, q);
  }

  /**
   * \brief Converts a rotation vector to an axis-angle representation.
   *
   * \param rv    The rotation vector.
   * \param axis  The normalised axis of rotation.
   * \param angle The angle of rotation (in radians).
   */
  template <typename T>
  static void rotation_vector_to_axis_angle(const T *rv, T *axis, T *angle)
  {
    const T minval = 1e-20f;
    const size_t elementCount = 3;
    T magnitude = l2_norm(rv, elementCount);

    // If the magnitude is close to zero then this corresponds to a zero rotation around an arbitrary.
    // Here we return a zero rotation around the x-axis.
    if(magnitude > minval)
    {
      *angle = magnitude;

      for(size_t i = 0; i < elementCount; ++i)
      {
        axis[i] = rv[i] / magnitude;
      }
    }
    else
    {
      *angle = 0.0f;
      axis[0] = 1.0f;
      axis[1] = 0.0f;
      axis[2] = 0.0f;
    }

  }

  /**
   * \brief Converts a rotation vector to a quaternion.
   *
   * \param rv  The rotation vector.
   * \param q   The unit quaternion.
   */
  template <typename T>
  static void rotation_vector_to_quaternion(const T *rv, T *q)
  {
    T axis[3];
    T angle;
    rotation_vector_to_axis_angle(rv, axis, &angle);

    // Create the quaternion.
    T halfAngle = angle / 2.0f;
    T sinHalfAngle = sin(halfAngle);
    q[0] = cos(halfAngle);
    q[1] = axis[0] * sinHalfAngle;
    q[2] = axis[1] * sinHalfAngle;
    q[3] = axis[2] * sinHalfAngle;
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
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

  /**
   * \brief Converts a unit quaternion to a rotation matrix arranged in row-major format.
   *
   * \param q      The unit quaternion.
   * \param matrix The rotation matrix in row-major format.
   */
  template <typename T>
  static void quaternion_to_row_major_rotation_matrix(const T *q, T *matrix)
  {
    T q0Sq = q[0]*q[0];
    T q1Sq = q[1]*q[1];
    T q2Sq = q[2]*q[2];
    T q3Sq = q[3]*q[3];

    matrix[0] = q0Sq + q1Sq - q2Sq - q3Sq;
    matrix[1] = 2.0f * (q[1]*q[2] + q[0]*q[3]);
    matrix[2] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    matrix[3] = 2.0f * (q[1]*q[2] - q[0]*q[3]);
    matrix[4] = q0Sq - q1Sq + q2Sq - q3Sq;
    matrix[5] = 2.0f * (q[2]*q[3] + q[0]*q[1]);
    matrix[6] = 2.0f * (q[1]*q[3] + q[0]*q[2]);
    matrix[7] = 2.0f * (q[2]*q[3] - q[0]*q[1]);
    matrix[8] = q0Sq - q1Sq - q2Sq + q3Sq;
  }

  /**
   * \brief Converts a rotation matrix arranged in row-major format to a unit quaternion.
   *
   * \param matrix   The rotation matrix in row-major format.
   * \param q        The unit quaternion.
   */
  template <typename T>
  static void row_major_rotation_matrix_to_quaternion(const T *matrix, T *q)
  {
    int variant = 0;

    // Choose the best variant.
    if(     (matrix[4] > -matrix[8]) && (matrix[0] > -matrix[4]) && (matrix[0] > -matrix[8])) variant = 0;
    else if((matrix[4] < -matrix[8]) && (matrix[0] >  matrix[4]) && (matrix[0] >  matrix[8])) variant = 1;
    else if((matrix[4] >  matrix[8]) && (matrix[0] <  matrix[4]) && (matrix[0] < -matrix[8])) variant = 2;
    else if((matrix[4] <  matrix[8]) && (matrix[0] < -matrix[4]) && (matrix[0] <  matrix[8])) variant = 3;

    double denominator;

    switch(variant)
    {
      case 0:
        denominator =  2.0f * sqrt(1.0f + matrix[0] + matrix[4] + matrix[8]);
        q[0] = denominator / 4.0f;
        q[1] = (matrix[5] - matrix[7]) / denominator;
        q[2] = (matrix[6] - matrix[2]) / denominator;
        q[3] = (matrix[1] - matrix[3]) / denominator;
        break;

      case 1:
        denominator =  2.0f * sqrt(1.0f + matrix[0] - matrix[4] - matrix[8]);
        q[0] = (matrix[5] - matrix[7]) / denominator;
        q[1] = denominator / 4.0f;
        q[2] = (matrix[1] + matrix[3]) / denominator;
        q[3] = (matrix[6] + matrix[2]) / denominator;
        break;

      case 2:
        denominator = 2.0f * sqrt(1.0f - matrix[0] + matrix[4] - matrix[8]);
        q[0] = (matrix[6] - matrix[2]) / denominator;
        q[1] = (matrix[1] + matrix[3]) / denominator;
        q[2] = denominator / 4.0f;
        q[3] = (matrix[5] + matrix[7]) / denominator;
        break;

      case 3:
        denominator = 2.0f * sqrt(1.0f - matrix[0] - matrix[4] + matrix[8]);
        q[0] = (matrix[1] - matrix[3]) / denominator;
        q[1] = (matrix[6] + matrix[2]) / denominator;
        q[2] = (matrix[5] + matrix[7]) / denominator;
        q[3] = denominator / 4.0f;
        break;
    }
  }

  /**
   * \brief Calculates the transpose of a 3x3 matrix.
   *
   * \param matrix          The original matrix.
   * \param matrixTranspose The transpose of the original matrix.
   */
  template <typename T>
  static void transpose_matrix3(const T *matrix, T *matrixTranspose)
  {
    std::copy(matrix, matrix + 9, matrixTranspose);

    // Swap off-diagonal elements.
    transpose_matrix3_in_place(matrixTranspose);
  }

  /**
   * \brief Overwrites a 3x3 matrix with its transpose.
   *
   * \param matrix  The matrix to transpose.
   */
  template <typename T>
  static void transpose_matrix3_in_place(T *matrix)
  {
    std::swap(matrix[1], matrix[3]);
    std::swap(matrix[2], matrix[6]);
    std::swap(matrix[5], matrix[7]);
  }
};

}

#endif
