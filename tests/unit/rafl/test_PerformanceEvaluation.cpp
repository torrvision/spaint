#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <rafl/examples/UnitCircleExampleGenerator.h>
#include <rafl/evaluation/PerformanceEvaluation.h>
using namespace rafl;

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

BOOST_AUTO_TEST_SUITE(test_PerformanceEvaluation)

BOOST_AUTO_TEST_CASE(get_conf_mtx_test)
{
  // Generate a set of labels;
  std::set<int> classLabels = list_of(2)(5)(1)(9);

  // Generate examples at the four axial points on the unit circle.
  UnitCircleExampleGenerator<Label> generator(classLabels, 1234, 0.0f, 0.0f);

  // Generate dummy ground truth and predicted labels.
  std::vector<Example_CPtr> groundTruthExamples = generator.generate_examples(list_of(9)(1)(5), 5);
  std::vector<Example_CPtr> predictedExamples = generator.generate_examples(list_of(5)(2)(1), 5);

  // Generate a confusion matrix.
  Eigen::MatrixXf testConfMtx = PerfEval::get_conf_mtx(classLabels, groundTruthExamples, predictedExamples);

  // The std::set orders the integers from smallest to largest.
  // The ground truth examples will be [1 5 9 1 5 9 ... 1 5 9]
  // The predicted labels will be      [1 2 5 1 2 5 ... 1 2 5]
  // Therefore the 1's are all in the correct place, i.e. (0,0) = 1.0f.
  // All the 5's are predicted as 2's.
  // Since the 5's are in the 3rd position in classLabels,
  // and the 2's are in the 2nd position,
  // we expect the all the pairs of labels [gt=5 pred=2]
  // to be at position (3-1,2-1), i.e. (2,1) = 1.0f.
  Eigen::MatrixXf trueConfMtx = Eigen::MatrixXf::Zero(4,4);
  trueConfMtx(0,0) = 1.0f;
  trueConfMtx(2,1) = 1.0f;
  trueConfMtx(3,2) = 1.0f;

  BOOST_CHECK_EQUAL(testConfMtx, trueConfMtx);
}

BOOST_AUTO_TEST_SUITE_END()
