/**
 * touchtrain: TouchTrainDataset.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TOUCHTRAIN_TOUCHTRAINDATASET
#define H_TOUCHTRAIN_TOUCHTRAINDATASET

#include <fstream>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <tvgutil/persistence/IOUtil.h>

#include "LabelledPath.h"

/**
 * \brief An instance of an instantiation of this class template represents a disk-based dataset for touchtrain.
 */
template <typename Label>
class TouchTrainDataset
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The directory in which to store the tables of results generated during cross-validation. */
  std::string m_crossValidationResultsDir;

  /** The directory in which to store the output models (i.e. the random forests). */
  std::string m_modelsDir;

  /** The root directory of the dataset. */
  std::string m_rootDir;

  /** The paths to the training images, together with their associated labels. */
  std::vector<LabelledPath<Label> > m_trainingImagePaths;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a touchtrain dataset.
   *
   * \param root            The root directory of the dataset.
   * \param sequenceNumbers The sequence numbers to be included during training.
   */
  TouchTrainDataset(const std::string& rootDir, const std::vector<size_t>& sequenceNumbers)
  : m_rootDir(rootDir)
  {
    // Maintain a count of the files and directories that are unexpectedly not found.
    size_t invalidCount = 0;

    // Find the cross-validation results directory.
    m_crossValidationResultsDir = rootDir + "/crossvalidation-results";
    if(!check_path_exists(m_crossValidationResultsDir)) ++invalidCount;

    // Find the models directory.
    m_modelsDir = rootDir + "/models";
    if(!check_path_exists(m_modelsDir)) ++invalidCount;

    // Find the sequence directories and check that they have the correct contents.
    boost::format threeDigits("%03d");
    for(size_t i = 0, size = sequenceNumbers.size(); i < size; ++i)
    {
      // Find the sequence directory itself.
      std::string sequenceDir = rootDir + "/seq" + (threeDigits % sequenceNumbers[i]).str();
      if(!check_path_exists(sequenceDir)) ++invalidCount;

      // Check that it contains an images directory and a file containing the image labels.
      std::string imagesDir = sequenceDir + "/images";
      std::string annotationFile = sequenceDir + "/annotation.txt";
      if(!check_path_exists(imagesDir)) ++invalidCount;
      if(!check_path_exists(annotationFile)) ++invalidCount;

      // Generate a set of labelled paths for the sequence's training images and check that it is non-empty.
      std::vector<LabelledPath<Label> > sequenceTrainingImagePaths = generate_labelled_image_paths(imagesDir, annotationFile);
      if(sequenceTrainingImagePaths.empty())
      {
        std::cout << "[touchtrain] Expecting some data in: " << sequenceDir << std::endl;
        ++invalidCount;
      }

      // Append the paths for the current sequence to the global set.
      m_trainingImagePaths.insert(m_trainingImagePaths.end(), sequenceTrainingImagePaths.begin(), sequenceTrainingImagePaths.end());
    }

    // If any of the expected data was missing, throw an error.
    if(invalidCount > 0)
    {
      throw std::runtime_error("Error: The aforementioned files and directories were not found, please create and populate them");
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

  /**
   * \brief Gets the paths to the training images, together with their associated labels.
   *
   * \return  The paths to the training images, together with their associated labels.
   */
  const std::vector<LabelledPath<Label> >& get_training_image_paths() const
  {
    return m_trainingImagePaths;
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Checks whether a specified path exists.
   *
   * If the path is not found, we output it using std::cout.
   *
   * \param path  The path.
   * \return      true, if the path exists, or false otherwise.
   */
  static bool check_path_exists(const std::string& path)
  {
    if(!boost::filesystem::exists(path))
    {
      std::cout << "[touchtrain] Expecting to see: " << path << std::endl;
      return false;
    }

    return true;
  }

  /**
   * \brief Generates a labelled path for each image in the specified images directory.
   *        The labels for the various images are supplied in a separate annotation file.
   *
   * \param imagesDir       The path to the images directory.
   * \param annotationFile  The path to a file containing the labels to associate with the images in the images directory.
   *
   * Each line in the annotation file is assumed to be in the following format: <imageName,label>
   *
   * \return  The labelled paths for all images in the specified images directory.
   */
  static std::vector<LabelledPath<Label> > generate_labelled_image_paths(const std::string& imagesDir, const std::string& annotationFile)
  {
    // FIXME: Make this robust to bad data.

    std::vector<LabelledPath<Label> > result;

    std::ifstream fs(annotationFile.c_str());
    if(!fs) throw std::runtime_error("Error: The file '" + annotationFile + "' could not be opened");

    const std::string delimiters(", \r");
    std::vector<std::vector<std::string> > wordLines = tvgutil::IOUtil::extract_word_lines(fs, delimiters);

    for(size_t i = 0, lineCount = wordLines.size(); i < lineCount; ++i)
    {
      const std::vector<std::string>& words = wordLines[i];
      if(words.size() != 2)
      {
        throw std::runtime_error("Error: Line " + boost::lexical_cast<std::string>(i) + " is not in the format <imageName,label>");
      }

      const std::string& imageFilename = words[0];
      Label label = boost::lexical_cast<Label>(words[1]);
      result.push_back(LabelledPath<Label>(imagesDir + "/" + imageFilename, label));
    }

    return result;
  }
};

#endif
