/**
 * rafl: PerfUtil.h
 */

#ifndef H_TVGUTIL_PERFUTIL
#define H_TVGUTIL_PERFUTIL

#include <set>
#include <map>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "../examples/Example.h"

namespace rafl {

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
  static float get_accuracy(const Eigen::MatrixXf confusion_matrix)
  {
    return confusion_matrix.trace()/confusion_matrix.sum();
  }

  /**
   * \brief A function which calculates a confusion matrix from a set of ground-truth and predicted label vectors.
   *
   * \param classLabels    Contains the entire set of currently known class labels.
   * \param groundTruth    A vector of labels which are assumed to be correct.
   * \param predicted      A vector of labels predicted by a machine.
   * \return               The generated confusion matrix.
   */
  template <typename Label>
  static Eigen::MatrixXf get_confusion_matrix(const std::set<Label>& classLabels, const std::vector<Label>& groundTruth, const std::vector<Label>& predicted)
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
   * \brief A convenience function which calculates a confusion matrix from a set of ground-truth and predicted example vectors.
   *
   * \param classLabels    Contains the entire set of currently known class labels.
   * \param groundTruth    A vector of examples with assumed correct labels.
   * \param predicted      A vector of the same examples with labels predicted by a machine.
   * \return               The generated confusion matrix.
   */
  template <typename Label>
  static Eigen::MatrixXf get_confusion_matrix(const std::set<Label>& classLabels, const std::vector<boost::shared_ptr<const Example<Label> > > groundTruth, const std::vector<boost::shared_ptr<const Example<Label> > > predicted)
  {
    assert(groundTruth.size() == predicted.size());

    std::vector<Label> gt;
    std::vector<Label> pred;

    for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
    {
      gt.push_back( groundTruth.at(i)->get_label() );
      pred.push_back( predicted.at(i)->get_label() );
    }

    return get_confusion_matrix(classLabels, gt, pred);
  }

  /**
   * \brief This function generates a matrix whose rows sum to one by dividing each row by its L1-norm.
   *
   * \param matrix   The matrix.
   * \return      The row-L1-normalised matrix.
   */
  static Eigen::MatrixXf normalise_rows_L1(const Eigen::MatrixXf matrix)
  {
    Eigen::MatrixXf matrixNormL1 = Eigen::MatrixXf::Zero(matrix.rows(), matrix.cols());
    for(size_t i = 0, iend = matrix.rows(); i < iend; ++i)
    {
      float rowL1Norm = matrix.row(i).lpNorm<1>();
      if(rowL1Norm > 0) matrixNormL1.row(i) = matrix.row(i) / rowL1Norm;
    }

    return matrixNormL1;
  }

};

}

#endif
