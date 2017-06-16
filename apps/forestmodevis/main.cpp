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
  std::string animationBaseName;
};

std::vector<std::string> drawModes(const std::vector<Mode> &modes, const std::string &nameBase, const Color &color, Viz3d &visualizer)
{
  std::vector<std::string> widgetNames;

  for(size_t modeIdx = 0; modeIdx < modes.size(); ++modeIdx)
  {
    const Mode &mode = modes[modeIdx];
    const std::string modeName =nameBase + '_' + boost::lexical_cast<std::string>(modeIdx);
    widgetNames.push_back(modeName);

    WCovarianceEllipsoid ellipsoid(mode.position, mode.covariance, color);
    visualizer.showWidget(modeName, ellipsoid);
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
      std::vector<std::string> modeNames = drawModes(cookie->modesByTree[treeIdx], "wModes_", cookie->treeColours[treeIdx], visualizer);

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

    fs::path currentModeFileName = cookie->pathGenerator->make_path(cookie->animationBaseName);

    while(fs::is_regular_file(currentModeFileName))
    {
      std::vector<std::vector<Mode>> currentModes;
      readModes(currentModeFileName.string(), currentModes);

      for(size_t treeIdx = 2; treeIdx < currentModes.size(); ++treeIdx)
      {
        std::vector<std::string> modeNames = drawModes(currentModes[treeIdx], "wModes_", cookie->treeColours[treeIdx], visualizer);

        fs::path screenshotPath = cookie->pathGenerator->make_path("modes_" + boost::lexical_cast<std::string>(treeIdx) + '_' + cookie->animationBaseName + ".png");
        std::cout << "Saving screenshot: " << screenshotPath << '\n';

        visualizer.saveScreenshot(screenshotPath.string());

        for(const auto &x : modeNames)
        {
          visualizer.removeWidget(x);
        }
      }

      cookie->pathGenerator->increment_index();
      currentModeFileName = cookie->pathGenerator->make_path(cookie->animationBaseName);
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
    cookie.animationBaseName = argv[3];
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
