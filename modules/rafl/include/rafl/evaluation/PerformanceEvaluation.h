/**
 * rafl: PerformanceEvaluation.h
 */

#ifndef H_TVGUTIL_PERFORMANCEEVALUATION
#define H_TVGUTIL_PERFORMANCEEVALUATION

#include <set>
#include <map>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "../examples/Example.h"

namespace rafl {

class PerfEval
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief A funciton which calculates a confusion matrix from a set of ground-truth and predicted example vectors.
   *
   * \param classLabels    Contains the entire set of currently known class labels.
   * \param groundTruth    A vector of examples with assumed correct labels.
   * \param predicted      A vector of the same examples with labels predcited by a machine.
   *
   * \return               The confusion matrix.
   */
  template <typename Label>
  static Eigen::MatrixXf get_conf_mtx(const std::set<Label>& classLabels, const std::vector<boost::shared_ptr<const Example<Label> > > groundTruth, const std::vector<boost::shared_ptr<const Example<Label> > > predicted)
  {
    assert(groundTruth.size() == predicted.size());

    std::vector<Label> gt;
    std::vector<Label> pred;

    for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
    {
      gt.push_back( groundTruth.at(i)->get_label() );
      pred.push_back( predicted.at(i)->get_label() );
    }

    return get_conf_mtx(classLabels, gt, pred);
  }

  /**
   * \brief A funciton which calculates a confusion matrix from a set of ground-truth and predicted label vectors.
   *
   * \param classLabels    Contains the entire set of currently known class labels.
   * \param groundTruth    A vector of labels with assumed correct labels.
   * \param predicted      A vector of the same labels with labels predcited by a machine.
   *
   * \return               The confusion matrix.
   */
  template <typename Label>
  static Eigen::MatrixXf get_conf_mtx(const std::set<Label>& classLabels, const std::vector<Label>& groundTruth, const std::vector<Label>& predicted)
  {
    assert(groundTruth.size() == predicted.size());

    size_t sizeLabels = classLabels.size();
    Eigen::MatrixXf confusionMatrix = Eigen::MatrixXf::Zero(sizeLabels, sizeLabels);

    std::map<Label,int> label2int;
    int i = 0;
    for(typename std::set<Label>::const_iterator it = classLabels.begin(), iend = classLabels.end(); it != iend; ++it)
    {
      label2int.insert(std::make_pair(*it, i));
      ++i;
    }

    for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
    {
      ++confusionMatrix(label2int[ groundTruth.at(i) ], label2int[ predicted.at(i) ]);
    }

    return confusionMatrix;
  }

  static Eigen::MatrixXf L1norm_mtx_rows(const Eigen::MatrixXf mtx)
  {
    Eigen::MatrixXf mtx_L1normalised_rows = Eigen::MatrixXf::Zero(mtx.rows(),mtx.cols());
    for(size_t i = 0, iend = mtx.rows(); i < iend; ++i)
    {
      float rowL1Norm = mtx.row(i).lpNorm<1>();
      if(rowL1Norm > 0) mtx_L1normalised_rows.row(i) = mtx.row(i) / rowL1Norm;
    }

    return mtx_L1normalised_rows;
  }

  static float get_accuracy(const Eigen::MatrixXf confusion_matrix)
  {
    return confusion_matrix.trace()/confusion_matrix.sum();
  }

};

}

#endif
