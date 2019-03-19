/**
 * spaintgui: Sequence.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_SPAINTGUI_SEQUENCE
#define H_SPAINTGUI_SEQUENCE

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <InputSource/ImageSourceEngine.h>

/**
 * \brief An instance of a class deriving from this one represents an RGB-D disk sequence.
 */
class Sequence
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The number of the initial frame that we want to use. */
  size_t m_initialFrameNumber;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a sequence.
   *
   * \param initialFrameNumber  The number of the initial frame that we want to use.
   */
  explicit Sequence(size_t initialFrameNumber);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the sequence.
   */
  virtual ~Sequence();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes the configuration for a tracker that will read the poses of the RGB-D frames in the sequence from disk.
   *
   * \return  The tracker configuration.
   */
  virtual std::string make_disk_tracker_config() const = 0;

  /**
   * \brief Makes an image source engine that will yield the RGB-D frames in the sequence.
   *
   * \param calibrationFilename The location of the calibration file to use.
   * \return                    The image source engine.
   */
  virtual InputSource::ImageSourceEngine *make_image_source_engine(const std::string& calibrationFilename) const = 0;

  /**
   * \brief Gets a representative string that specifies the sequence.
   *
   * \return  A representative string that specifies the sequence.
   */
  virtual std::string to_string() const = 0;

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the path to the disk directory containing the sequence.
   *
   * \return  The path to the disk directory containing the sequence.
   */
  virtual boost::filesystem::path dir() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the default location of the sequence's calibration file.
   *
   * \note  This file may or may not actually be present.
   *
   * \return The default location of the sequence's calibration file.
   */
  boost::filesystem::path default_calib_path() const;

  /**
   * \brief Gets the ID of the sequence.
   *
   * \return  The ID of the sequence.
   */
  std::string id() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const Sequence> Sequence_CPtr;

//#################### STREAM OPERATORS ####################

extern std::ostream& operator<<(std::ostream& os, const Sequence& sequence);

#endif
