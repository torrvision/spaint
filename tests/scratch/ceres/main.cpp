#include <ceres/ceres.h>

#include <ITMLib/Utils/ITMMath.h>
using namespace ORUtils;

struct CostFunctor
{
  Vector2d m_a, m_b;

  CostFunctor(const Vector2d& a, const Vector2d& b)
  : m_a(a), m_b(b)
  {}

  template <typename T>
  bool operator()(const T *const trans, T *residual) const
  {
    T dx = m_b[0] - (m_a[0] + trans[0]);
    T dy = m_b[1] - (m_a[1] + trans[1]);
    residual[0] = sqrt(dx * dx + dy * dy);
    return true;
  }
};

struct Callback : ceres::IterationCallback
{
  const Vector2d& m_trans;

  explicit Callback(const Vector2d& trans)
  : m_trans(trans)
  {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
  {
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t" << m_trans << '\n';
    return ceres::SOLVER_CONTINUE;
  }
};

int main(int argc, char *argv[])
{
  // Initialise glog.
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be mutated in place by the solver.
  Vector2d trans(0.0, 0.0);

  // Build the problem.
  ceres::Problem problem;
  Vector2d as[] = { Vector2d(1,1), Vector2d(2,2) };
  Vector2d bs[] = { Vector2d(2,3), Vector2d(3.5,4.5) };
  int count = sizeof(as) / sizeof(Vector2d);
  for(int i = 0; i < count; ++i)
  {
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<CostFunctor, 1, 2>(new CostFunctor(as[i], bs[i]));
    problem.AddResidualBlock(costFunction, NULL, trans.v);
  }

  // Set up the solver.
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;

  // Add an iteration callback to monitor how the parameters change.
  Callback callback(trans);
  options.callbacks.push_back(&callback);

  // Run the solver.
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Output a report.
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Trans: " << trans << '\n';

  return 0;
}
