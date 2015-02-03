#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/util/ConfusionMatrixUtil.h>
using namespace evaluation;

#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

//#################### HELPERS ####################

/**
 * \brief Makes a confusion matrix from a set of ground truth and predicted example vectors.
 *
 * \param classLabels    The entire set of currently known class labels.
 * \param groundTruth    A vector of examples with assumed correct labels.
 * \param predicted      A vector of the same examples with labels predicted by a machine.
 * \return               The generated confusion matrix.
 */
template <typename Label>
static Eigen::MatrixXf make_confusion_matrix(const std::set<Label>& classLabels, const std::vector<boost::shared_ptr<const Example<Label> > > groundTruth, const std::vector<boost::shared_ptr<const Example<Label> > > predicted)
{
  assert(groundTruth.size() == predicted.size());

  std::vector<Label> gt;
  std::vector<Label> pred;

  for(size_t i = 0, iend = groundTruth.size(); i < iend; ++i)
  {
    gt.push_back( groundTruth.at(i)->get_label() );
    pred.push_back( predicted.at(i)->get_label() );
  }

  return ConfusionMatrixUtil::make_confusion_matrix(classLabels, gt, pred);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ConfusionMatrixUtil)

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
  Eigen::MatrixXf testConfMtx = make_confusion_matrix(classLabels, groundTruthExamples, predictedExamples);

  // The std::set orders the integers from smallest to largest.
  // The ground truth examples will be [1 5 9 1 5 9 ... 1 5 9]
  // The predicted labels will be      [1 2 5 1 2 5 ... 1 2 5]
  // Therefore the 1's are all in the correct place, i.e. (0,0) = 5.0f.
  // All the 5's are predicted as 2's.
  // Since the 5's are in the 3rd position in classLabels,
  // and the 2's are in the 2nd position,
  // we expect the all the pairs of labels [gt=5 pred=2]
  // to be at position (3-1,2-1), i.e. (2,1) = 5.0f.
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
