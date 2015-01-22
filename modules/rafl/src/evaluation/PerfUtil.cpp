/**
 * rafl: PerfUtil.cpp
 */

#include "evaluation/PerfUtil.h"

namespace rafl {

float PerfUtil::get_accuracy(const Eigen::MatrixXf confusion_matrix)
{
  return confusion_matrix.trace()/confusion_matrix.sum();
}

Eigen::MatrixXf PerfUtil::normalise_rows_L1(const Eigen::MatrixXf matrix)
{
  Eigen::MatrixXf matrixNormL1 = Eigen::MatrixXf::Zero(matrix.rows(), matrix.cols());
  for(size_t i = 0, iend = matrix.rows(); i < iend; ++i)
  {
    float rowL1Norm = matrix.row(i).lpNorm<1>();
    if(rowL1Norm > 0) matrixNormL1.row(i) = matrix.row(i) / rowL1Norm;
  }

  return matrixNormL1;
}

}

