/**
 * spaintgui: SpaintSequence.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_SPAINTGUI_SPAINTSEQUENCE
#define H_SPAINTGUI_SPAINTSEQUENCE

#include "Sequence.h"

/**
 * \brief An instance of this class represents an RGB-D disk sequence that is stored in the spaint format.
 */
class SpaintSequence : public Sequence
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The mask used to specify the sequence's depth images. */
  std::string m_depthImageMask;

  /** The sigma of the Gaussian to use when corrupting the depth with zero-mean, depth-dependent Gaussian noise (0 = disabled). */
  float m_depthNoiseSigma;

  /** The disk directory containing the sequence. */
  boost::filesystem::path m_dir;

  /** The fraction of the depth images to zero out (in the range [0,1]). */
  double m_missingDepthFraction;

  /** The mask used to specify the sequence's pose files. */
  std::string m_poseFileMask;

  /** The mask used to specify the sequence's RGB images. */
  std::string m_rgbImageMask;

  /** The mask used to specify the sequence's semantic images. */
  std::string m_semanticImageMask;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an spaint sequence.
   *
   * \param dir                   The disk directory containing the sequence.
   * \param initialFrameNumber    The number of the initial frame that we want to use.
   * \param missingDepthFraction  The fraction of the depth images to zero out (in the range [0,1]).
   * \param depthNoiseSigma       The sigma of the Gaussian to use when corrupting the depth with zero-mean, depth-dependent Gaussian noise (0 = disabled).
   */
  SpaintSequence(const boost::filesystem::path& dir, size_t initialFrameNumber, double missingDepthFraction, float depthNoiseSigma);

  /**
   * \brief Constructs an spaint sequence.
   *
   * \param depthImageMask        The mask used to specify the sequence's depth images.
   * \param rgbImageMask          The mask used to specify the sequence's RGB images.
   * \param poseFileMask          The mask used to specify the sequence's pose files.
   * \param semanticImageMask     The mask used to specify the sequence's semantic images.
   * \param initialFrameNumber    The number of the initial frame that we want to use.
   * \param missingDepthFraction  The fraction of the depth images to zero out (in the range [0,1]).
   * \param depthNoiseSigma       The sigma of the Gaussian to use when corrupting the depth with zero-mean, depth-dependent Gaussian noise (0 = disabled).
   */
  SpaintSequence(const std::string& depthImageMask, const std::string& rgbImageMask, const std::string& poseFileMask, const std::string& semanticImageMask,
                 size_t initialFrameNumber, double missingDepthFraction, float depthNoiseSigma);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::string make_disk_tracker_config() const;

  /** Override */
  virtual InputSource::ImageSourceEngine *make_image_source_engine(const std::string& calibrationFilename) const;

  /** Override */
  virtual std::string to_string() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual boost::filesystem::path dir() const;
};

#endif
