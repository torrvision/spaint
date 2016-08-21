/**
 * fitvis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <ceres/ceres.h>

#include <ITMLib/Utils/ITMMath.h>
using namespace ORUtils;

#include <tvgplot/PaletteGenerator.h>
#include <tvgplot/PlotWindow.h>
using namespace tvgplot;

//#################### GLOBAL VARIABLES ####################

Vector2d as[] = { Vector2d(1,1), Vector2d(2,2) };
Vector2d bs[] = { Vector2d(2,3), Vector2d(3.5,4.5) };

std::map<std::string,cv::Scalar> palette = PaletteGenerator::generate_basic_rgba_palette();

//#################### TYPES ####################

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
  const PlotWindow& m_plot;
  const Vector2d& m_trans;

  Callback(PlotWindow& plot, const Vector2d& trans)
  : m_plot(plot), m_trans(trans)
  {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
  {
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t" << m_trans << '\n';

    m_plot.clear_figure();
    m_plot.draw_cartesian_axes(palette["White"]);
    int count = sizeof(as) / sizeof(Vector2d);
    for(int i = 0; i < count; ++i)
    {
      m_plot.draw_cartesian_circle(cv::Point2f(as[i].x, as[i].y), palette["Red"], 10);
      Vector2d b = bs[i] - m_trans;
      m_plot.draw_cartesian_circle(cv::Point2f(b.x, b.y), palette["Blue"], 10);
    }
    m_plot.refresh();

    if(summary.iteration == 0 || summary.step_is_successful)
    {
      cv::waitKey();
    }

    return ceres::SOLVER_CONTINUE;
  }
};

//#################### FUNCTIONS ####################

int main(int argc, char *argv[])
{
  // Initialise glog.
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be mutated in place by the solver.
  Vector2d trans(0.0, 0.0);

  // Build the problem.
  ceres::Problem problem;
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

  // Set up the plot window in which we will display the fitting process.
  PlotWindow plot("Fitting Visualisation", 700, 700, 20);

  // Add an iteration callback to monitor how the parameters change.
  Callback callback(plot, trans);
  options.callbacks.push_back(&callback);

  // Run the solver.
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Output a report.
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Trans: " << trans << '\n';

  cv::waitKey();

  return 0;
}
