/**
 * evaluation: ConfusionMatrixUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "util/ConfusionMatrixUtil.h"

namespace evaluation {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float ConfusionMatrixUtil::calculate_accuracy(const Eigen::MatrixXf& confusionMatrix)
{
  return confusionMatrix.trace() / confusionMatrix.sum();
}

Eigen::MatrixXf ConfusionMatrixUtil::normalise_rows_L1(const Eigen::MatrixXf& matrix)
{
  Eigen::MatrixXf matrixNormL1 = Eigen::MatrixXf::Zero(matrix.rows(), matrix.cols());
  for(size_t i = 0, rowCount = matrix.rows(); i < rowCount; ++i)
  {
    float rowL1Norm = matrix.row(i).lpNorm<1>();
    if(rowL1Norm > 0) matrixNormL1.row(i) = matrix.row(i) / rowL1Norm;
  }
  return matrixNormL1;
}

}
