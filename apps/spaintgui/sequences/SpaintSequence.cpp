/**
 * spaintgui: SpaintSequence.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "SpaintSequence.h"
using namespace InputSource;

#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
namespace bf = boost::filesystem;
using boost::assign::list_of;

#include <itmx/imagesources/DepthCorruptingImageSourceEngine.h>
#include <itmx/imagesources/SemanticMaskingImageSourceEngine.h>
using namespace itmx;

//#################### CONSTRUCTORS ####################

SpaintSequence::SpaintSequence(const bf::path& dir, size_t initialFrameNumber, double missingDepthFraction, float depthNoiseSigma)
: Sequence(initialFrameNumber), m_depthNoiseSigma(depthNoiseSigma), m_dir(dir), m_missingDepthFraction(missingDepthFraction)
{
  // Try to figure out the format of the sequence stored in the directory (we only check the depth images, since the colour ones might be missing).
  const bool sevenScenesNaming = bf::is_regular_file(dir / "frame-000000.depth.png");
  const bool spaintNaming = bf::is_regular_file(dir / "depthm000000.pgm");

  // Determine the depth/RGB/pose masks.
  if(sevenScenesNaming && spaintNaming)
  {
    throw std::runtime_error("Error: The directory '" + dir.string() + "' contains images that follow both the 7-Scenes and spaint naming conventions.");
  }
  else if(sevenScenesNaming)
  {
    m_depthImageMask = (dir / "frame-%06i.depth.png").string();
    m_poseFileMask = (dir / "frame-%06i.pose.txt").string();
    m_rgbImageMask = (dir / "frame-%06i.color.png").string();
    m_semanticImageMask = (dir / "frame-%06i.semantic.png").string();
  }
  else if(spaintNaming)
  {
    m_depthImageMask = (dir / "depthm%06i.pgm").string();
    m_poseFileMask = (dir / "posem%06i.txt").string();
    m_rgbImageMask = (dir / "rgbm%06i.ppm").string();
    m_semanticImageMask = "";
  }
  else
  {
    throw std::runtime_error("Error: The directory '" + dir.string() + "' does not contain depth images that follow a known naming convention.");
  }
}

SpaintSequence::SpaintSequence(const std::string& depthImageMask, const std::string& rgbImageMask, const std::string& poseFileMask, const std::string& semanticImageMask,
                               size_t initialFrameNumber, double missingDepthFraction, float depthNoiseSigma)
: Sequence(initialFrameNumber),
  m_depthImageMask(depthImageMask),
  m_depthNoiseSigma(depthNoiseSigma),
  m_dir(bf::path(depthImageMask).parent_path()),
  m_missingDepthFraction(missingDepthFraction),
  m_poseFileMask(poseFileMask),
  m_rgbImageMask(rgbImageMask),
  m_semanticImageMask(semanticImageMask)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::string SpaintSequence::make_disk_tracker_config() const
{
  return "<tracker type='infinitam'><params>type=file,mask=" + m_poseFileMask + ",initialFrameNo=" +
         boost::lexical_cast<std::string>(m_initialFrameNumber) + "</params></tracker>";
}

ImageSourceEngine *SpaintSequence::make_image_source_engine(const std::string& calibrationFilename) const
{
  if(m_semanticImageMask != "")
  {
    ImageMaskPathGenerator normalPathGenerator(m_rgbImageMask.c_str(), m_depthImageMask.c_str());
    ImageMaskPathGenerator semanticPathGenerator(m_semanticImageMask.c_str(), m_depthImageMask.c_str());
    ImageSourceEngine *normalSource = new ImageFileReader<ImageMaskPathGenerator>(calibrationFilename.c_str(), normalPathGenerator, m_initialFrameNumber);
    ImageSourceEngine *semanticSource = new ImageFileReader<ImageMaskPathGenerator>(calibrationFilename.c_str(), semanticPathGenerator, m_initialFrameNumber);

    if(bf::exists(semanticPathGenerator.getRgbImagePath(m_initialFrameNumber)))
    {
      return new SemanticMaskingImageSourceEngine(normalSource, semanticSource, SemanticMaskingImageSourceEngine::MASK_DEPTH_ONLY, list_of("person")("sky"));
    }
  }

  ImageMaskPathGenerator pathGenerator(m_rgbImageMask.c_str(), m_depthImageMask.c_str());
  ImageSourceEngine *imageFileReader = new ImageFileReader<ImageMaskPathGenerator>(calibrationFilename.c_str(), pathGenerator, m_initialFrameNumber);
  return m_missingDepthFraction > 0.0 || m_depthNoiseSigma > 0.0f ? new DepthCorruptingImageSourceEngine(imageFileReader, m_missingDepthFraction, m_depthNoiseSigma) : imageFileReader;
}

std::string SpaintSequence::to_string() const
{
  return m_rgbImageMask + " " + m_depthImageMask;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

bf::path SpaintSequence::dir() const
{
  return m_dir;
}
