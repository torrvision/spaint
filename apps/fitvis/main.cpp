/**
 * fitvis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <cmath>

#include <ceres/ceres.h>

#include <ITMLib/Utils/ITMMath.h>
using namespace ORUtils;

#include <tvgplot/PaletteGenerator.h>
#include <tvgplot/PlotWindow.h>
using namespace tvgplot;

//#################### GLOBAL VARIABLES ####################

Vector2d as[] = { Vector2d(1,1), Vector2d(1,2), Vector2d(2,1) };
Vector2d bs[] = { Vector2d(2,3), Vector2d(3,3), Vector2d(2,2) };

std::map<std::string,cv::Scalar> palette = PaletteGenerator::generate_basic_rgba_palette();

//#################### TYPES ####################

struct CostFunctor
{
  Vector2d m_a, m_b;

  CostFunctor(const Vector2d& a, const Vector2d& b)
  : m_a(a), m_b(b)
  {}

  template <typename T>
  bool operator()(const T *const theta, const T *const trans, T *residual) const
  {
    T cosTheta = cos(*theta);
    T sinTheta = sin(*theta);
    T transformedAx = m_a[0] * cosTheta - m_a[1] * sinTheta + trans[0];
    T transformedAy = m_a[0] * sinTheta + m_a[1] * cosTheta + trans[1];
    T dx = m_b[0] - transformedAx;
    T dy = m_b[1] - transformedAy;
    residual[0] = sqrt(dx * dx + dy * dy);
    return true;
  }
};

struct Callback : ceres::IterationCallback
{
  const PlotWindow& m_plot;
  const double& m_theta;
  const Vector2d& m_trans;

  Callback(PlotWindow& plot, const double& theta, const Vector2d& trans)
  : m_plot(plot), m_theta(theta), m_trans(trans)
  {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
  {
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t" << m_theta << "; " << m_trans << '\n';

    m_plot.clear_figure();
    m_plot.draw_cartesian_axes(palette["White"]);
    int count = sizeof(as) / sizeof(Vector2d);
    for(int i = 0; i < count; ++i)
    {
      m_plot.draw_cartesian_circle(cv::Point2f(bs[i].x, bs[i].y), palette["Blue"], 10);

      double cosTheta = cos(m_theta);
      double sinTheta = sin(m_theta);
      Vector2d a(
        as[i].x * cosTheta - as[i].y * sinTheta + m_trans.x,
        as[i].x * sinTheta + as[i].y * cosTheta + m_trans.y
      );
      m_plot.draw_cartesian_circle(cv::Point2f(a.x, a.y), palette["Red"], 10);
    }
    m_plot.refresh();

    if(summary.iteration == 0 || summary.step_is_successful)
    {
      if(cv::waitKey() == 'q') return ceres::SOLVER_ABORT;
    }

    return ceres::SOLVER_CONTINUE;
  }
};

//#################### FUNCTIONS ####################

int main(int argc, char *argv[])
{
  // Initialise glog.
  google::InitGoogleLogging(argv[0]);

  // The variables to solve for with their initial values. They will be mutated in place by the solver.
  double theta = 0.0;
  Vector2d trans(0.0, 0.0);

  // Build the problem.
  ceres::Problem problem;
  int count = sizeof(as) / sizeof(Vector2d);
  for(int i = 0; i < count; ++i)
  {
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 2>(new CostFunctor(as[i], bs[i]));
    problem.AddResidualBlock(costFunction, NULL, &theta, trans.v);
  }

  // Set up the solver.
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;

  // Set up the plot window in which we will display the fitting process.
  PlotWindow plot("Fitting Visualisation", 700, 700, 20);

  // Add an iteration callback to monitor how the parameters change.
  Callback callback(plot, theta, trans);
  options.callbacks.push_back(&callback);

  // Run the solver.
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Output a report.
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Trans: " << trans << '\n';

  if(summary.termination_type != ceres::USER_FAILURE)
  {
    cv::waitKey();
  }

  return 0;
}
