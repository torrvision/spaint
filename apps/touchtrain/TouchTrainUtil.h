/**
 * touchtrain: TouchTrainUtil.h
 */

#ifndef H_TOUCHTRAINUTIL
#define H_TOUCHTRAINUTIL

#include <boost/lexical_cast.hpp>

#include <rafl/examples/Example.h>

#include <spaint/touch/TouchDescriptorCalculator.h>

#include <tvgutil/WordExtractor.h>

/**
 * \brief This class contains functions that help us to generate labelled image paths and generate examples from them.
 */
struct TouchTrainUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

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
  template <typename Label>
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

  /**
   * \brief Generates an array of examples given an array of labelled image paths.
   *
   * \param labelledImagePaths  The array of labelled image paths.
   * \return                    The examples.
   */
  template <typename Label>
  static std::vector<boost::shared_ptr<const rafl::Example<Label> > > generate_examples(const std::vector<LabelledPath<Label> >& labelledImagePaths)
  {
    typedef boost::shared_ptr<const rafl::Example<Label> > Example_CPtr;
    int labelledImagePathCount = static_cast<int>(labelledImagePaths.size());
    std::vector<Example_CPtr> result(labelledImagePathCount);

    for(int i = 0; i < labelledImagePathCount; ++i)
    {
        af::array img = af::loadImage(labelledImagePaths[i].path.c_str());
        rafl::Descriptor_CPtr descriptor = spaint::TouchDescriptorCalculator::calculate_histogram_descriptor(img);
        result[i].reset(new rafl::Example<Label>(descriptor, labelledImagePaths[i].label));
    }

    return result;
  }
};

#endif
