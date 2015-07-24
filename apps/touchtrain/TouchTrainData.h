/**
 * touchtrain: TouchTrainData.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TOUCHTRAIN_TOUCHTRAINDATA
#define H_TOUCHTRAIN_TOUCHTRAINDATA

#include <fstream>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <tvgutil/WordExtractor.h>

#include "LabelledPath.h"

/**
 * \brief An instance of an instantiation of this class template represents a disk-based dataset for touchtrain.
 */
template <typename Label>
class TouchTrainData
{
  //#################### TYPEDEFS ####################
private:
  typedef std::vector<LabelledPath<Label> > LabelledImagePaths;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The directory in which to store the tables of results generated during cross-validation. */
  std::string m_crossValidationResultsDir;

  /** An array of labelled image paths. */
  LabelledImagePaths m_labelledImagePaths;

  /** The directory in which to store the output models (i.e. the random forests). */
  std::string m_modelsDir;

  /** The root directory of the dataset. */
  std::string m_rootDir;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs the paths and data relevant for touch training.
   *
   * \param root             The root directory of the dataset.
   * \param sequenceNumbers  An array containing the sequence numbers to be included during training.
   */
  TouchTrainData(const std::string& rootDir, const std::vector<size_t>& sequenceNumbers)
  : m_rootDir(rootDir)
  {
    size_t invalidCount = 0;

    m_crossValidationResultsDir = rootDir + "/crossvalidation-results";
    if(!check_path_exists(m_crossValidationResultsDir)) ++invalidCount;

    m_modelsDir = rootDir + "/models";
    if(!check_path_exists(m_modelsDir)) ++invalidCount;

    boost::format threeDigits("%03d");
    for(size_t i = 0, size = sequenceNumbers.size(); i < size; ++i)
    {
      std::string sequencePath = rootDir + "/seq" + (threeDigits % sequenceNumbers[i]).str();
      if(!check_path_exists(sequencePath)) ++invalidCount;

      std::string imagePath = sequencePath + "/images";
      std::string annotationPath = sequencePath + "/annotation.txt";
      if(!check_path_exists(imagePath)) ++invalidCount;
      if(!check_path_exists(annotationPath)) ++invalidCount;

      LabelledImagePaths labelledImagePathSet = generate_labelled_image_paths(imagePath, annotationPath);

      if(labelledImagePathSet.empty())
      {
        std::cout << "[touchtrain] Expecting some data in: " << sequencePath << std::endl;
        ++invalidCount;
      }

      // Append the labelled image paths from the current sequence directory to the global set.
      m_labelledImagePaths.insert(m_labelledImagePaths.end(), labelledImagePathSet.begin(), labelledImagePathSet.end());
    }

    if(invalidCount > 0)
    {
      throw std::runtime_error("The aforementioned directories were not found, please create and populate them.");
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the directory in which to store the tables of results generated during cross-validation.
   *
   * \return  The directory in which to store the tables of results generated during cross-validation.
   */
  const std::string& get_cross_validation_results_directory() const
  {
    return m_crossValidationResultsDir;
  }

  /**
   * \brief TODO
   */
  const LabelledImagePaths& get_labelled_image_paths() const
  {
    return m_labelledImagePaths;
  }

  /**
   * \brief Gets the directory in which to store the output models.
   *
   * \return  The directory in which to store the output models.
   */
  const std::string& get_models_directory() const
  {
    return m_modelsDir;
  }

  /**
   * \brief Gets the root directory of the dataset.
   *
   * \return  The root directory of the dataset.
   */
  const std::string& get_root_directory() const
  {
    return m_rootDir;
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Checks whether a specified path exists.
   *
   * If the path is not found, it outputs the expected path to std::cout.
   *
   * \param path  The path.
   * \return      True, if the path exists, false otherwise.
   */
  static bool check_path_exists(const std::string& path)
  {
    if(!boost::filesystem::exists(path))
    {
      std::cout << "[touchtrain] Expecting to see: " << path << std::endl;
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * \brief Generates a labelled path for each image in the specified images directory.
   *        The labels for the various images are supplied in a separate annotation file.
   *
   * \param imagesPath      The path to the images directory.
   * \param annotationPath  The path to a file containing the labels to associate with the images in the images path.
   *
   * The annotation is assumed to be in the following format: <imageName,label>
   *
   * \return   The labelled paths for all images in the specified images directory.
   */
  static std::vector<LabelledPath<Label> > generate_labelled_image_paths(const std::string& imagesPath, const std::string& annotationPath)
  {
    // FIXME: Make this robust to bad data.

    std::vector<LabelledPath<Label> > labelledImagePaths;

    std::ifstream fs(annotationPath.c_str());
    if(!fs) throw std::runtime_error("The file: " + annotationPath + " could not be opened.");

    const std::string delimiters(", \r");
    std::vector<std::vector<std::string> > wordLines = tvgutil::WordExtractor::extract_word_lines(fs, delimiters);

    for(size_t i = 0, lineCount = wordLines.size(); i < lineCount; ++i)
    {
      const std::vector<std::string>& words = wordLines[i];
      const std::string& imageFilename = words[0];
      Label label = boost::lexical_cast<Label>(words.back());
      labelledImagePaths.push_back(LabelledPath<Label>(imagesPath + "/" + imageFilename, label));
    }

    return labelledImagePaths;
  }
};

#endif
