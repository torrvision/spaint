/**
 * evaluation: ConfusionMatrixUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_CONFUSIONMATRIXUTIL
#define H_EVALUATION_CONFUSIONMATRIXUTIL

#include <cassert>
#include <map>
#include <set>
#include <vector>

#include <Eigen/Dense>

namespace evaluation {

/**
 * \brief This struct provides utility functions for working with confusion matrices.
 */
struct ConfusionMatrixUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Calculates the accuracy of a learner from a confusion matrix.
   *
   * The accuracy value is defined to be the trace of the confusion matrix divided by the sum of its elements.
   *
   * \param confusionMtx   The confusion matrix.
   * \return               The accuracy value for the learner.
   */
  static float calculate_accuracy(const Eigen::MatrixXf& confusionMatrix);

  /**
   * \brief Makes a confusion matrix from a set of ground truth and predicted label vectors.
   *
   * \param classLabels    The entire set of currently-known class labels.
   * \param groundTruth    A vector of labels which are assumed to be correct.
   * \param predicted      A vector of labels predicted by a machine.
   * \return               The generated confusion matrix.
   */
  template <typename Label>
  static Eigen::MatrixXf make_confusion_matrix(const std::set<Label>& classLabels, const std::vector<Label>& groundTruth, const std::vector<Label>& predicted)
  {
    assert(groundTruth.size() == predicted.size());

    size_t sizeLabels = classLabels.size();
    Eigen::MatrixXf confusionMatrix = Eigen::MatrixXf::Zero(sizeLabels, sizeLabels);

    // Generate consecutive numeric indices for the various labels (these will be used to index into the confusion matrix).
    std::map<Label,int> labelToIndex;
    int i = 0;
    for(typename std::set<Label>::const_iterator it = classLabels.begin(), iend = classLabels.end(); it != iend; ++it)
    {
      labelToIndex.insert(std::make_pair(*it, i));
      ++i;
    }

    // Fill in the confusion matrix.
    for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
    {
      ++confusionMatrix(labelToIndex[groundTruth[i]], labelToIndex[predicted[i]]);
    }

    return confusionMatrix;
  }

  /**
   * \brief Generates a "row-L1-normalised" copy of a matrix in which each row sums to one.
   *
   * This is achieved by dividing each row in the input matrix by its L1 norm.
   *
   * \param matrix  The input matrix.
   * \return        The row-L1-normalised copy of the matrix.
   */
  static Eigen::MatrixXf normalise_rows_L1(const Eigen::MatrixXf& matrix);
};

}

#endif
