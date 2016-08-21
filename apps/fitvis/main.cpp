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

struct Element
{
  Vector2d pos;
  cv::Scalar colour;

  Element(const Vector2d& pos_, const cv::Scalar& colour_)
  : pos(pos_), colour(colour_)
  {}
};

std::map<std::string,cv::Scalar> palette = PaletteGenerator::generate_basic_rgba_palette();

Element as[] = {
  Element(Vector2d(2,0), palette["Yellow"]),
  Element(Vector2d(2,1), palette["Green"]),
  Element(Vector2d(1,1), palette["Red"]),
  Element(Vector2d(1,2), palette["Blue"])
};

Element bs[] = {
  Element(Vector2d(3,2), palette["Yellow"]),
  Element(Vector2d(4,2), palette["Green"]),
  Element(Vector2d(4,3), palette["Red"]),
  Element(Vector2d(5,3), palette["Blue"])
};

int correspondences[] = { 3, 2, 1, 0 };

//#################### TYPES ####################

struct PositionCostFunctor
{
  int m_i;

  PositionCostFunctor(int i)
  : m_i(i)
  {}

  template <typename T>
  bool operator()(const T *const theta, const T *const trans, T *residuals) const
  {
    T cosTheta = cos(*theta);
    T sinTheta = sin(*theta);
    T transformedAx = aPos().x * cosTheta - aPos().y * sinTheta + trans[0];
    T transformedAy = aPos().x * sinTheta + aPos().y * cosTheta + trans[1];
    T dx = bPos()[0] - transformedAx;
    T dy = bPos()[1] - transformedAy;
    residuals[0] = sqrt(dx * dx + dy * dy);
    return true;
  }

  const Vector2d& aPos() const { return as[m_i].pos; }
  const Vector2d& bPos() const { return bs[correspondences[m_i]].pos; }
};

struct ManualPositionCostFunction : ceres::SizedCostFunction<1, 1, 2>
{
  int m_i;

  ManualPositionCostFunction(int i)
  : m_i(i)
  {}

  virtual ~ManualPositionCostFunction() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
  {
    const double *theta = parameters[0];
    const double *trans = parameters[1];

    double cosTheta = cos(*theta);
    double sinTheta = sin(*theta);
    double transformedAx = aPos().x * cosTheta - aPos().y * sinTheta + trans[0];
    double transformedAy = aPos().x * sinTheta + aPos().y * cosTheta + trans[1];
    double dx = bPos()[0] - transformedAx;
    double dy = bPos()[1] - transformedAy;
    residuals[0] = sqrt(dx * dx + dy * dy);

    if(jacobians != NULL && jacobians[0] != NULL)
    {
      jacobians[0][0] = (dx * (aPos().x * sinTheta + aPos().y * cosTheta) + dy * (aPos().y * sinTheta - aPos().x * cosTheta)) / residuals[0];
      jacobians[1][0] = -dx / residuals[0];
      jacobians[1][1] = -dy / residuals[0];
    }
    return true;
  }

  const Vector2d& aPos() const { return as[m_i].pos; }
  const Vector2d& bPos() const { return bs[correspondences[m_i]].pos; }
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
    int count = sizeof(as) / sizeof(*as);
    for(int i = 0; i < count; ++i)
    {
      m_plot.draw_cartesian_square(cv::Point2f(bs[i].pos.x, bs[i].pos.y), bs[i].colour, 10, 1);

      double cosTheta = cos(m_theta);
      double sinTheta = sin(m_theta);
      Vector2d a(
        as[i].pos.x * cosTheta - as[i].pos.y * sinTheta + m_trans.x,
        as[i].pos.x * sinTheta + as[i].pos.y * cosTheta + m_trans.y
      );

      m_plot.draw_cartesian_circle(cv::Point2f(a.x, a.y), as[i].colour, 10);
    }
    m_plot.refresh();

    update_correspondences();

    if(summary.iteration == 0 || summary.step_is_successful)
    {
      if(cv::waitKey() == 'q') return ceres::SOLVER_ABORT;
    }

    return ceres::SOLVER_CONTINUE;
  }

  void update_correspondences()
  {
    int count = sizeof(as) / sizeof(*as);
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tCorrespondences: ";
    for(int i = 0; i < count; ++i)
    {
      std::cout << correspondences[i] << ' ';
    }
    std::cout << "-> ";


    for(int i = 0; i < count; ++i)
    {
      double bestCost = INT_MAX;
      int bestIndex = -1;

      for(int j = 0; j < count; ++j)
      {
        double dx = as[i].pos.x - bs[j].pos.x;
        double dy = as[i].pos.y - bs[j].pos.y;
        double dr = as[i].colour[0] - bs[j].colour[0];
        double dg = as[i].colour[1] - bs[j].colour[1];
        double db = as[i].colour[2] - bs[j].colour[2];
        double posWeight = 1.0;
        double colourWeight = 5.0;
        double cost = posWeight * (dx * dx + dy * dy) + colourWeight * (dr * dr + dg * dg + db * db);
        if(cost < bestCost)
        {
          bestCost = cost;
          bestIndex = j;
        }
      }

      correspondences[i] = bestIndex;
      std::cout << correspondences[i] << ' ';
    }

    std::cout << '\n';
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
  int count = sizeof(as) / sizeof(*as);
  for(int i = 0; i < count; ++i)
  {
    //ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<PositionCostFunctor, 1, 1, 2>(new PositionCostFunctor(i));
    ceres::CostFunction *costFunction = new ManualPositionCostFunction(i);
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
  std::cout << summary.FullReport() << "\n";
  std::cout << "Theta: " << theta << '\n';
  std::cout << "Trans: " << trans << '\n';

  if(summary.termination_type != ceres::USER_FAILURE)
  {
    cv::waitKey();
  }

  return 0;
}
