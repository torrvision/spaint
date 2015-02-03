/**
 * evaluation: PerfUtil.h
 */

#ifndef H_EVALUATION_PERFUTIL
#define H_EVALUATION_PERFUTIL

#include <cassert>
#include <map>
#include <set>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace evaluation {

class PerfUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the accuracy from a confusion matrix.
   *
   * \param confusionMtx   The confusion matrix.
   * \return               The trace of the confusion matrix divided by the sum of its elements.
   */
  static float get_accuracy(const Eigen::MatrixXf& confusion_matrix);

  /**
   * \brief Makes a confusion matrix from a set of ground-truth and predicted label vectors.
   *
   * \param classLabels    Contains the entire set of currently known class labels.
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

    //This mapping is required to generate consecutive indices into the confusion matrix from an arbitrary set of labels.
    std::map<Label,int> label2int;
    int i = 0;
    for(typename std::set<Label>::const_iterator it = classLabels.begin(), iend = classLabels.end(); it != iend; ++it)
    {
      label2int.insert(std::make_pair(*it, i));
      ++i;
    }

    for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
    {
      ++confusionMatrix(label2int[groundTruth.at(i)], label2int[predicted.at(i)]);
    }

    return confusionMatrix;
  }

  /**
   * \brief Generates a "row-L1-normalised" copy of a matrix in which each row sums to one.
   *
   * This is achieved by dividing each row in the input matrix by its L1 norm.
   *
   * \param matrix  The input matrix.
   * \return        The row-L1-normalised copy.
   */
  static Eigen::MatrixXf normalise_rows_L1(const Eigen::MatrixXf& matrix);
};

}

#endif
