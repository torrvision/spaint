#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/util/ConfusionMatrixUtil.h>
using namespace evaluation;

//#################### TYPEDEFS ####################

typedef int Label;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ConfusionMatrixUtil)

BOOST_AUTO_TEST_CASE(get_conf_mtx_test)
{
  // Generate a set of labels;
  std::set<Label> classLabels = list_of(2)(5)(1)(9);

  // Generate dummy ground truth and predicted labels.
  std::vector<Label> groundTruthLabels = list_of(1)(5)(9)(1)(5)(9)(1)(5)(9)(1)(5)(9)(1)(5)(9);
  std::vector<Label> predictedLabels = list_of(1)(2)(5)(1)(2)(5)(1)(2)(5)(1)(2)(5)(1)(2)(5);

  // Generate a confusion matrix.
  Eigen::MatrixXf testConfMtx = ConfusionMatrixUtil::make_confusion_matrix(classLabels, groundTruthLabels, predictedLabels);

  // The std::set orders the integers from smallest to largest.
  // The ground truth examples will be [1 5 9 1 5 9 ... 1 5 9]
  // The predicted labels will be      [1 2 5 1 2 5 ... 1 2 5]
  // Therefore the 1's are all in the correct place, i.e. trueConfMtx(0,0) = 5.0f.
  // All the 5's are predicted as 2's.
  // Since the 5's are in the 3rd position in classLabels,
  // and the 2's are in the 2nd position,
  // we expect the all the pairs of labels [gt=5 pred=2]
  // to be at position (3-1,2-1), i.e. trueConfMtx(2,1) = 5.0f.
  Eigen::MatrixXf trueConfMtx = Eigen::MatrixXf::Zero(4,4);
  trueConfMtx(0,0) = 5.0f;
  trueConfMtx(2,1) = 5.0f;
  trueConfMtx(3,2) = 5.0f;

  BOOST_CHECK_EQUAL(testConfMtx, trueConfMtx);

  Eigen::MatrixXf testConfMtxNormL1 = ConfusionMatrixUtil::normalise_rows_L1(trueConfMtx);
  Eigen::MatrixXf trueConfMtxNormL1 = Eigen::MatrixXf::Zero(4,4);
  trueConfMtxNormL1(0,0) = 1.0f;
  trueConfMtxNormL1(2,1) = 1.0f;
  trueConfMtxNormL1(3,2) = 1.0f;

  BOOST_CHECK_EQUAL(testConfMtxNormL1, trueConfMtxNormL1);
}

BOOST_AUTO_TEST_SUITE_END()
