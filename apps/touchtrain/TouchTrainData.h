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
 * \brief A struct that represents the file structure for the touch training data.
 */
template <typename Label>
class TouchTrainData
{
  //#################### TYPEDEFS ####################
private:
  typedef std::vector<LabelledPath<Label> > LabelledImagePaths;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The directory containing tables of results generated during cross-validation. */
  std::string m_crossValidationResults;

  /** An array of labelled image paths. */
  LabelledImagePaths m_labelledImagePaths;

  /** The directory where the random forest models are stored. */
  std::string m_models;

  /** The root directory in which the touch training data is stored. */
  std::string m_root;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs the paths and data relevant for touch training.
   *
   * \param root             The root directory containing the touch training data.
   * \param sequenceNumbers  An array containing the sequence numbers to be included during training.
   */
  TouchTrainData(const std::string& root, const std::vector<size_t>& sequenceNumbers)
  : m_root(root)
  {
    size_t invalidCount = 0;

    m_crossValidationResults = root + "/crossvalidation-results";
    if(!check_path_exists(m_crossValidationResults)) ++invalidCount;

    m_models = root + "/models";
    if(!check_path_exists(m_models)) ++invalidCount;

    boost::format threeDigits("%03d");
    for(size_t i = 0, size = sequenceNumbers.size(); i < size; ++i)
    {
      std::string sequencePath = root + "/seq" + (threeDigits % sequenceNumbers[i]).str();
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
