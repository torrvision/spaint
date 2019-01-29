/**
 * orx: GeometryUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ORX_GEOMETRYUTIL
#define H_ORX_GEOMETRYUTIL

#include <cmath>
#include <map>
#include <vector>

#include <Eigen/Dense>

#include <ORUtils/Math.h>
#include <ORUtils/SE3Pose.h>

#include "DualQuaternion.h"

namespace orx {

/**
 * \brief This struct provides a number of useful geometric utility functions.
 */
struct GeometryUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Linearly blends a set of poses together to construct a refined pose.
   *
   * \param poses The poses to blend.
   * \return      The refined pose.
   */
  static ORUtils::SE3Pose blend_poses(const std::vector<ORUtils::SE3Pose>& poses);

  /**
   * \brief Converts a dual quaternion to an SE(3) pose.
   *
   * \param dq  The dual quaternion.
   * \return    The corresponding SE(3) pose.
   */
  template <typename T>
  static ORUtils::SE3Pose dual_quat_to_pose(const DualQuaternion<T>& dq)
  {
    ORUtils::SE3Pose pose;

    const Vector3f r = dq.get_rotation().toFloat();
    const Vector3f t = dq.get_translation().toFloat();

    pose.SetR(to_rotation_matrix(r));
    pose.SetT(t);

    return pose;
  }

  /**
   * \brief Estimates the rigid body transformation from three 3D points in a set P to three corresponding 3D points in a set Q using the Kabsch algorithm.
   *
   * Specifically, we estimate M such that M * P.col(i).homogeneous() is as close as possible to Q.col(i).homogeneous() for every i.
   *
   * \param P The first set of 3D points (each point is a column in the matrix).
   * \param Q The second set of 3D points (each point is a column in the matrix).
   * \return  The estimated rigid body transformation from each point in P to its corresponding point in Q.
   */
  static Eigen::Matrix4f estimate_rigid_transform(const Eigen::Matrix3f& P, const Eigen::Matrix3f& Q);

  /**
   * \brief Estimates the rigid body transformation from three 3D points in a set P to three corresponding 3D points in a set Q using the Kabsch algorithm.
   *
   * Specifically, we estimate M such that M * P.col(i).homogeneous() is as close as possible to Q.col(i).homogeneous() for every i.
   *
   * \param P The first set of 3D points (each point is a column in the matrix).
   * \param Q The second set of 3D points (each point is a column in the matrix).
   * \param R A location into which to write the estimated rotation matrix.
   * \param t A location into which to write the estimated translation vector.
   */
  static void estimate_rigid_transform(const Eigen::Matrix3f& P, const Eigen::Matrix3f& Q, Eigen::Matrix3f& R, Eigen::Vector3f& t);

  /**
   * \brief Finds a pose hypothesis with the greatest number of inliers from a set of such hypotheses.
   *
   * \param poseHypotheses            The set of pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \param rotThreshold              The angular threshold to use when comparing rotations.
   * \param transThreshold            The distance threshold to use when comparing translations.
   * \return                          The best hypothesis.
   */
  static ORUtils::SE3Pose find_best_hypothesis(const std::vector<ORUtils::SE3Pose>& poseHypotheses,
                                               std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                               double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);

  /**
   * \brief Finds a pose hypothesis with the greatest number of inliers from a set of such hypotheses.
   *
   * \param poseHypotheses            The set of pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \param rotThreshold              The angular threshold to use when comparing rotations.
   * \param transThreshold            The distance threshold to use when comparing translations.
   * \return                          The ID of the best hypothesis.
   */
  static std::string find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& poseHypotheses,
                                          std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                          double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);

  /**
   * \brief Converts an SE(3) pose to a dual quaternion.
   *
   * \param pose  The SE(3) pose.
   * \return      The corresponding dual quaternion.
   */
  template <typename T>
  static DualQuaternion<T> pose_to_dual_quat(const ORUtils::SE3Pose& pose)
  {
    const Vector3f r = to_rotation_vector(pose.GetR());
    const Vector3f t = pose.GetT();

    ORUtils::Vector3<T> typedR(static_cast<T>(r.x), static_cast<T>(r.y), static_cast<T>(r.z));
    ORUtils::Vector3<T> typedT(static_cast<T>(t.x), static_cast<T>(t.y), static_cast<T>(t.z));

    return DualQuaternion<T>::from_translation(typedT) * DualQuaternion<T>::from_rotation(typedR);
  }

  /**
   * \brief Determines whether or not two SE(3) poses are sufficiently similar.
   *
   * Similarity is defined in terms of both the rotations and translations involved. Rotation similarity is
   * assessed by looking at the relative rotation mapping one of the two input rotations to the other, and
   * thresholding the angle involved. Translation similarity is assessed by thresholding the distance between
   * the two input translations. Iff both their rotations and translations are similar, so are the poses.
   *
   * \param pose1           The first pose.
   * \param pose2           The second pose.
   * \param rotThreshold    The angular threshold to use when comparing the rotations.
   * \param transThreshold  The distance threshold to use when comparing the translations.
   * \return                true, if the poses are sufficiently similar, or false otherwise.
   */
  static bool poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);

  /**
   * \brief Converts an InfiniTAM matrix to an Eigen matrix.
   *
   * \param M  The InfiniTAM matrix.
   * \return   The Eigen matrix.
   */
  template <typename T>
  static Eigen::Matrix<T,3,3> to_eigen(const ORUtils::Matrix3<T>& M)
  {
    Eigen::Matrix<T,3,3> result;

    for(int row = 0; row < 3; ++row)
    {
      for(int col = 0; col < 3; ++col)
      {
        result(row, col) = M(col, row);
      }
    }

    return result;
  }

  /**
   * \brief Converts an InfiniTAM vector to an Eigen vector.
   *
   * \param v  The InfiniTAM vector.
   * \return   The Eigen vector.
   */
  template <typename T>
  static Eigen::Matrix<T,3,1> to_eigen(const ORUtils::Vector3<T>& v)
  {
    return Eigen::Matrix<T,3,1>(v.x, v.y, v.z);
  }

  /**
   * \brief Converts a rotation vector to a rotation matrix.
   *
   * \param r The rotation vector.
   * \return  The corresponding rotation matrix.
   */
  template <typename T, typename U = T>
  static ORUtils::Matrix3<U> to_rotation_matrix(const ORUtils::Vector3<T>& r)
  {
    ORUtils::Matrix3<T> R;

    T angleSquared = ORUtils::dot(r, r);
    if(angleSquared > 1e-6)
    {
      T angle = sqrt(angleSquared);
      ORUtils::Vector3<T> axis = r / angle;
      R = to_itm(Eigen::Matrix<T,3,3>(Eigen::AngleAxis<T>(angle, to_eigen(axis))));
    }
    else R.setIdentity();

    return ORUtils::Matrix3<U>(R.m);
  }

  /**
   * \brief Converts a rotation matrix to a rotation vector.
   *
   * \param R The rotation matrix.
   * \return  The corresponding rotation vector.
   */
  template <typename T, typename U = T>
  static ORUtils::Vector3<U> to_rotation_vector(const ORUtils::Matrix3<T>& R)
  {
    Eigen::AngleAxis<T> aa(to_eigen(R));
    ORUtils::Vector3<T> r = to_itm(Eigen::Matrix<T,3,1>(aa.angle() * aa.axis()));
    return ORUtils::Vector3<U>(static_cast<U>(r.x), static_cast<U>(r.y), static_cast<U>(r.z));
  }

  /**
   * \brief Converts an Eigen matrix to an InfiniTAM matrix.
   *
   * \param M  The Eigen matrix.
   * \return   The InfiniTAM matrix.
   */
  template <typename T>
  static ORUtils::Matrix3<T> to_itm(const Eigen::Matrix<T,3,3>& M)
  {
    ORUtils::Matrix3<T> result;

    for(int row = 0; row < 3; ++row)
    {
      for(int col = 0; col < 3; ++col)
      {
        result(col, row) = M(row, col);
      }
    }

    return result;
  }

  /**
   * \brief Converts an Eigen vector to an InfiniTAM vector.
   *
   * \param v  The Eigen vector.
   * \return   The InfiniTAM vector.
   */
  template <typename T>
  static ORUtils::Vector3<T> to_itm(const Eigen::Matrix<T,3,1>& v)
  {
    return ORUtils::Vector3<T>(v[0], v[1], v[2]);
  }

  /**
   * \brief Converts an InfiniTAM matrix into a string representation of it that can be copied into Matlab.
   *
   * \param M An InfiniTAM matrix.
   * \return  A string representation of the InfiniTAM matrix that can be copied into Matlab.
   */
  static std::string to_matlab(const Matrix4f& M);
};

}

#endif
