/**
 * forestmodevis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz/widget_accessor.hpp>
#include <sstream>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkActor.h>
#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/timing/TimeUtil.h"

namespace fs = boost::filesystem;
using namespace cv::viz;

std::vector<Color> treeColors
{ Color::red(), Color::green(), Color::blue(), Color::magenta(), Color::cyan() };

struct Mode
{
  cv::Vec3f position;
  cv::Matx33f covariance;
};

class WCovarianceEllipsoid: public Widget3D
{
public:
  WCovarianceEllipsoid(const cv::Vec3f &position, const cv::Matx33f &covariance,
      Color color = Color::white())
  {
    // Compute the transformation parameters
    cv::Mat eVals;
    cv::Mat eVecs;
    cv::eigen(covariance, eVals, eVecs);
    cv::sqrt(eVals, eVals);

    auto scalingTransform = Eigen::Scaling(std::sqrt(covariance.val[0]),
        std::sqrt(covariance.val[4]), std::sqrt(covariance.val[8]));
//    auto scalingTransform = Eigen::Scaling(
//        Eigen::Map<Eigen::Vector3f>(eVals.ptr<float>()));
    auto rotation = Eigen::AngleAxisf(
        Eigen::Map<Eigen::Matrix3f>(eVecs.ptr<float>()));
    auto translation = Eigen::Translation3f(
        Eigen::Map<const Eigen::Vector3f>(position.val));

    Eigen::Affine3f affineTransform = translation * scalingTransform * rotation;

// row major
    Eigen::Matrix4d transformMat = affineTransform.matrix().transpose().cast<
        double>();

    vtkSmartPointer < vtkSphereSource > sphereSource = vtkSmartPointer
        < vtkSphereSource > ::New();
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetRadius(1.0);

    vtkSmartPointer < vtkTransform > transform = vtkSmartPointer < vtkTransform
        > ::New();
    transform->SetMatrix(transformMat.data());

    vtkSmartPointer < vtkTransformPolyDataFilter > transformFilter =
        vtkSmartPointer < vtkTransformPolyDataFilter > ::New();
    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    // Set up the actor to display the transformed polydata

    vtkSmartPointer < vtkPolyDataMapper > transformedMapper = vtkSmartPointer
        < vtkPolyDataMapper > ::New();
    transformedMapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer < vtkActor > actor = vtkSmartPointer < vtkActor > ::New();
    actor->SetMapper(transformedMapper);
    actor->GetProperty()->SetColor(color.val[2] / 255, color.val[1] / 255,
        color.val[0] / 255);

    WidgetAccessor::setProp(*this, actor);
  }
};

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "Usage: " << argv[0] << " mesh.obj modes.txt\n";
    return 1;
  }

  const std::string meshFile = argv[1];
  const std::string modesFile = argv[2];

  // Load mesh
  Mesh mesh = Mesh::load(meshFile, Mesh::LOAD_OBJ);
  WMesh wMesh(mesh);

  // Load modes
  static const int NTREES = 5;
  // To visualize Kabsch modes
  // static const int NTREES = 3;
  std::vector<std::vector<Mode>> modesByTree(NTREES);

  std::ifstream inModes(modesFile);
  for (int treeIdx = 0; treeIdx < NTREES; ++treeIdx)
  {
    int nbModes, leafIdx;
    inModes >> nbModes >> leafIdx;
    for (int modeIdx = 0; modeIdx < nbModes; ++modeIdx)
    {
      int nbInliers;
      Mode mode;
      inModes >> nbInliers;
      for (int i = 0; i < 3; ++i)
        inModes >> mode.position.val[i];
      for (int i = 0; i < 9; ++i)
        inModes >> mode.covariance.val[i];

      modesByTree[treeIdx].push_back(mode);
    }
  }

  // Show everything
  Viz3d visualizer("Modes Visualizer");
  visualizer.showWidget("wMesh", wMesh);

  for (int treeIdx = 0; treeIdx < NTREES; ++treeIdx)
  {
    Color color = treeColors[treeIdx];
    for (size_t modeIdx = 0; modeIdx < modesByTree[treeIdx].size(); ++modeIdx)
    {
      const Mode &mode = modesByTree[treeIdx][modeIdx];
      const std::string modeName = "wMode_"
          + boost::lexical_cast<std::string>(treeIdx) + '_'
          + boost::lexical_cast<std::string>(modeIdx);

      const float radius = std::sqrt(
          std::max(std::max(mode.covariance.val[0], mode.covariance.val[4]),
              mode.covariance.val[8]));

      cv::Vec3f eVals;
      cv::Matx33f eVecs;
      cv::eigen(mode.covariance, eVals, eVecs);

      eVals.val[0] = sqrt(eVals.val[0]);
      eVals.val[1] = sqrt(eVals.val[1]);
      eVals.val[2] = sqrt(eVals.val[2]);

      cv::Affine3f wPose;
      wPose.translation(mode.position);
//      wPose.rotation(eVecs * eVals);

      cv::Point3d minPoint(-std::sqrt(mode.covariance.val[0]),
          -std::sqrt(mode.covariance.val[4]),
          -std::sqrt(mode.covariance.val[8]));
      cv::Point3d maxPoint = -minPoint;

      WCube cube(minPoint, maxPoint, false, color);
      WSphere sphere(cv::Point3d(), radius, 10, color);
      WCovarianceEllipsoid ellipsoid(mode.position, mode.covariance, color);

//      visualizer.showWidget(modeName, cube, wPose);
      visualizer.showWidget(modeName, ellipsoid);
    }
  }

//  WCovarianceEllipsoid test(cv::Vec3f(), cv::Matx33f(), Color::bluberry());
//  visualizer.showWidget("test", test);

  visualizer.spin();

  return 0;
}
