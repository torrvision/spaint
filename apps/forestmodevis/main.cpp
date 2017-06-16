/**
 * forestmodevis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <Eigen/Geometry>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widget_accessor.hpp>
#include <sstream>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/filesystem/SequentialPathGenerator.h"
#include "tvgutil/timing/TimeUtil.h"

namespace fs = boost::filesystem;
using namespace cv::viz;
using namespace tvgutil;

static const int NTREES = 5;

struct Mode
{
  cv::Vec3f position;
  cv::Matx33f covariance;
};

void readModes(const std::string &modesFile, std::vector<std::vector<Mode>> &modes)
{
  modes.clear();
  modes.resize(NTREES);

  std::ifstream inModes(modesFile);
  for(int treeIdx = 0; treeIdx < NTREES; ++treeIdx)
  {
    int nbModes, leafIdx;
    inModes >> nbModes >> leafIdx;
    for(int modeIdx = 0; modeIdx < nbModes; ++modeIdx)
    {
      int nbInliers;
      Mode mode;
      inModes >> nbInliers;
      for(int i = 0; i < 3; ++i) inModes >> mode.position.val[i];
      for(int i = 0; i < 9; ++i) inModes >> mode.covariance.val[i];

      modes[treeIdx].push_back(mode);
    }
  }
}

void readExamples(const std::string &examplesFile, std::vector<std::vector<cv::Point3d>> &examples)
{
  examples.clear();
  examples.resize(NTREES);

  std::ifstream inExamples(examplesFile);
  for(int treeIdx = 0; treeIdx < NTREES; ++treeIdx)
  {
    int nbExamples;
    inExamples >> nbExamples;
//    std::cout << examplesFile << " - Nb examples for tree " << treeIdx << ": " << nbExamples << '\n';

    for(int exampleIdx = 0; exampleIdx < nbExamples; ++exampleIdx)
    {
      cv::Point3d example;

      inExamples >> example.x >> example.y >> example.z;

//      std::cout << example << '\n';

      examples[treeIdx].push_back(example);
    }
  }
}

class WCovarianceEllipsoid : public Widget3D
{
public:
  WCovarianceEllipsoid(const cv::Vec3f &position, const cv::Matx33f &covariance, Color color = Color::white())
  {
    // Compute the transformation parameters
    cv::Mat eVals;
    cv::Mat eVecs;
    cv::eigen(covariance, eVals, eVecs);
    cv::sqrt(eVals, eVals);

    auto scalingTransform =
        Eigen::Scaling(std::sqrt(covariance.val[0]), std::sqrt(covariance.val[4]), std::sqrt(covariance.val[8]));
    //    auto scalingTransform = Eigen::Scaling(
    //        Eigen::Map<Eigen::Vector3f>(eVals.ptr<float>()));
    auto rotation = Eigen::AngleAxisf(Eigen::Map<Eigen::Matrix3f>(eVecs.ptr<float>()));
    auto translation = Eigen::Translation3f(Eigen::Map<const Eigen::Vector3f>(position.val));

    Eigen::Affine3f affineTransform = translation * scalingTransform * rotation;

    // row major
    Eigen::Matrix4d transformMat = affineTransform.matrix().transpose().cast<double>();

    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetRadius(1.0);

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(transformMat.data());

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    // Set up the actor to display the transformed polydata

    vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    transformedMapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(transformedMapper);
    actor->GetProperty()->SetColor(color.val[2] / 255, color.val[1] / 255, color.val[0] / 255);

    WidgetAccessor::setProp(*this, actor);
  }
};

struct VisualizationCookie
{
  Viz3d *visualizer;
  std::vector<Color> treeColours;
  std::vector<std::vector<Mode>> modesByTree;
  SequentialPathGenerator *pathGenerator;
  WMesh *mesh;
  std::string baseName;
  std::vector<std::string> widgetNames;
  std::string animationModesBaseName;
  std::string animationExamplesBaseName;
};

std::vector<std::string>
    drawModes(const std::vector<Mode> &modes, const std::string &nameBase, const Color &color, Viz3d &visualizer)
{
  std::vector<std::string> widgetNames;

  for(size_t modeIdx = 0; modeIdx < modes.size(); ++modeIdx)
  {
    const Mode &mode = modes[modeIdx];
    const std::string modeName = nameBase + '_' + boost::lexical_cast<std::string>(modeIdx);
    widgetNames.push_back(modeName);

    WCovarianceEllipsoid ellipsoid(mode.position, mode.covariance, color);
    visualizer.showWidget(modeName, ellipsoid);
  }

  return widgetNames;
}

std::vector<std::string> drawExamples(const std::vector<cv::Point3d> &examples,
                                      const std::string &nameBase,
                                      const Color &color,
                                      Viz3d &visualizer)
{
  std::vector<std::string> widgetNames;

  for(size_t exampleIdx = 0; exampleIdx < examples.size(); ++exampleIdx)
  {
    const cv::Point3d &example = examples[exampleIdx];

    const std::string exampleName = nameBase + '_' + boost::lexical_cast<std::string>(exampleIdx);
    widgetNames.push_back(exampleName);

    WSphere sphere(example, 0.01, 10, color);
    visualizer.showWidget(exampleName, sphere);
  }

  return widgetNames;
}

static void vizCallbackSingleModes(const KeyboardEvent &event, void *c)
{
  const VisualizationCookie *cookie = static_cast<VisualizationCookie *>(c);

  if(event.action == KeyboardEvent::KEY_UP && event.code == 't')
  {
    std::cout << "Saving screenshots from current viewpoint.\n";

    Viz3d &visualizer = *cookie->visualizer;

    // Clean the visualizer
    for(const auto &x : cookie->widgetNames)
    {
      visualizer.removeWidget(x);
    }

    for(size_t treeIdx = 2; treeIdx < cookie->modesByTree.size(); ++treeIdx)
    {
      std::vector<std::string> modeNames =
          drawModes(cookie->modesByTree[treeIdx], "wModes_", cookie->treeColours[treeIdx], visualizer);

      visualizer.saveScreenshot(cookie->baseName + "_" + boost::lexical_cast<std::string>(treeIdx) + ".png");

      for(const auto &x : modeNames)
      {
        visualizer.removeWidget(x);
      }
    }
  }
}

static void vizCallbackAnimation(const KeyboardEvent &event, void *c)
{
  const VisualizationCookie *cookie = static_cast<VisualizationCookie *>(c);

  if(event.action == KeyboardEvent::KEY_UP && event.code == 't')
  {
    std::cout << "Saving screenshots from current viewpoint.\n";

    Viz3d &visualizer = *cookie->visualizer;

    // Clean the visualizer
    for(const auto &x : cookie->widgetNames)
    {
      visualizer.removeWidget(x);
    }

    fs::path currentModeFileName = cookie->pathGenerator->make_path(cookie->animationModesBaseName);
    fs::path currentExamplesFileName = cookie->pathGenerator->make_path(cookie->animationExamplesBaseName);

    while(fs::is_regular_file(currentModeFileName))
    {
      std::vector<std::vector<Mode>> currentModes;
      readModes(currentModeFileName.string(), currentModes);

      std::vector<std::vector<cv::Point3d>> currentExamples;
      readExamples(currentExamplesFileName.string(), currentExamples);

      for(size_t treeIdx = 0; treeIdx < currentModes.size(); ++treeIdx)
      {
//        // Draw modes
//        {
//          std::vector<std::string> modeNames =
//              drawModes(currentModes[treeIdx], "wModes_", cookie->treeColours[treeIdx], visualizer);

//          fs::path screenshotPath = cookie->pathGenerator->make_path(
//              "modes_" + boost::lexical_cast<std::string>(treeIdx) + '_' + cookie->animationModesBaseName + ".png");
//          std::cout << "Saving screenshot: " << screenshotPath << '\n';

//          visualizer.saveScreenshot(screenshotPath.string());

//          for(const auto &x : modeNames)
//          {
//            visualizer.removeWidget(x);
//          }
//        }

//        // Draw examples
//        {
//          std::vector<std::string> exampleNames = drawExamples(currentExamples[treeIdx], "wExamples_", cookie->treeColours[treeIdx], visualizer);

//          fs::path screenshotPath = cookie->pathGenerator->make_path(
//              "examples_" + boost::lexical_cast<std::string>(treeIdx) + '_' + cookie->animationModesBaseName + ".png");
//          std::cout << "Saving screenshot: " << screenshotPath << '\n';

//          visualizer.saveScreenshot(screenshotPath.string());

//          for(const auto &x : exampleNames)
//          {
//            visualizer.removeWidget(x);
//          }
//        }

        // Draw combined
        {
          std::vector<std::string> exampleNames = drawExamples(currentExamples[treeIdx], "wExamples_", cookie->treeColours[treeIdx], visualizer);
          std::vector<std::string> modeNames = drawModes(currentModes[treeIdx], "wModes_", Color::gold(), visualizer);

          fs::path screenshotPath = cookie->pathGenerator->make_path(
              "combined_" + boost::lexical_cast<std::string>(treeIdx) + '_' + cookie->animationModesBaseName + ".png");
          std::cout << "Saving screenshot: " << screenshotPath << '\n';

          visualizer.saveScreenshot(screenshotPath.string());

          for(const auto &x : exampleNames)
          {
            visualizer.removeWidget(x);
          }

          for(const auto &x : modeNames)
          {
            visualizer.removeWidget(x);
          }
        }

      }

      cookie->pathGenerator->increment_index();
      currentModeFileName = cookie->pathGenerator->make_path(cookie->animationModesBaseName);
      currentExamplesFileName = cookie->pathGenerator->make_path(cookie->animationExamplesBaseName);
    }
  }
}

int main(int argc, char *argv[])
{
  if(argc < 3)
  {
    std::cout << "Usage: " << argv[0] << " mesh.obj modes.txt\n";
    return 1;
  }

  const std::string meshFile = argv[1];
  const std::string modesFile = argv[2];

  VisualizationCookie cookie;

  if(argc > 3)
  {
    cookie.animationModesBaseName = argv[3];
    cookie.animationExamplesBaseName = argv[4];
  }

  // Load mesh
  Mesh mesh = Mesh::load(meshFile, Mesh::LOAD_OBJ);
  WMesh wMesh(mesh);

  // Load modes
  // To visualize Kabsch modes
  // static const int NTREES = 3;

  readModes(modesFile, cookie.modesByTree);

  // Show everything
  Viz3d visualizer("Modes Visualizer");
  visualizer.showWidget("wMesh", wMesh);

  //  std::vector<Color> treeColors { Color::red(), Color::green(), Color::blue(), Color::magenta(), Color::cyan() };
  cookie.treeColours.push_back(Color::blue());
  cookie.treeColours.push_back(Color::green());
  cookie.treeColours.push_back(Color::red());
  cookie.treeColours.push_back(Color::magenta());
  cookie.treeColours.push_back(Color::cyan());

  // Here we draw all modes for visualization purpose.
  static const int INITIAL_TREE = 2;
  for(int treeIdx = INITIAL_TREE; treeIdx < NTREES; ++treeIdx)
  {
    Color color = cookie.treeColours[treeIdx];
    for(size_t modeIdx = 0; modeIdx < cookie.modesByTree[treeIdx].size(); ++modeIdx)
    {
      const Mode &mode = cookie.modesByTree[treeIdx][modeIdx];
      const std::string modeName =
          "wMode_" + boost::lexical_cast<std::string>(treeIdx) + '_' + boost::lexical_cast<std::string>(modeIdx);
      cookie.widgetNames.push_back(modeName);

      WCovarianceEllipsoid ellipsoid(mode.position, mode.covariance, color);
      visualizer.showWidget(modeName, ellipsoid);
    }
  }

  SequentialPathGenerator spg("./clusters");

  cookie.visualizer = &visualizer;
  cookie.pathGenerator = &spg;
  cookie.mesh = &wMesh;
  cookie.baseName = fs::path(modesFile).stem().string();

  //  visualizer.registerKeyboardCallback(vizCallbackSingleModes, &cookie);
  visualizer.registerKeyboardCallback(vizCallbackAnimation, &cookie);

  //  WCovarianceEllipsoid test(cv::Vec3f(), cv::Matx33f(), Color::bluberry());
  //  visualizer.showWidget("test", test);

  visualizer.spin();

  return 0;
}
