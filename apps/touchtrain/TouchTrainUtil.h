/**
 * touchtrain: TouchTrainUtil.h
 */

#ifndef H_TOUCHTRAINUTIL
#define H_TOUCHTRAINUTIL

#include <fstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <rafl/examples/Example.h>

#include <spaint/touch/TouchUtil.h>

/**
 * \brief A struct that represents a labelled image path.
 */
template <typename Label>
struct LabelledImagePath
{
  //#################### PUBLIC VARIABLES ####################

  /** The path to an image. */
  std::string m_imagePath;

  /** The label associated with the image path. */
  Label m_label;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs the labelled image path.
   *
   * \param imagePath  The path of the image.
   * \param label      The label associated with the specified image path.
   */
  LabelledImagePath(const std::string& imagePath, const Label& label)
  : m_imagePath(imagePath), m_label(label)
  {}
};

/**
 * \brief This class contains functions that help us to load labelled image paths and generate examples from them.
 */
struct TouchTrainUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Generates an array of labelled image paths.
   *
   * \param imagesPath      The path to the images directory.
   * \param annotationPath  The path to the specified file containing the labels associated with each image in the images path.
   *
   * The annotation is assumed to be in the following format: <imageName,label>
   *
   * \return  An array containing the labelled image paths.
   */
  template <typename Label>
  static std::vector<LabelledImagePath<Label> > generate_labelled_image_paths(const std::string& imagesPath, const std::string& annotationPath)
  {
    std::vector<LabelledImagePath<Label> > labelledImagePaths;

    std::ifstream fs(annotationPath.c_str());
    std::string line;
    while(std::getline(fs, line))
    {
      typedef boost::char_separator<char> sep;
      typedef boost::tokenizer<sep> tokenizer;
      tokenizer tok(line.begin(), line.end(), sep(", \r"));
      std::vector<std::string> tokens(tok.begin(), tok.end());

      std::string imageName = tokens[0];
      Label label = boost::lexical_cast<Label>(tokens.back());
      labelledImagePaths.push_back(LabelledImagePath<Label>(imagesPath + "/" + imageName, label));
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
  static std::vector<boost::shared_ptr<const Example<Label> > > generate_examples(const std::vector<LabelledImagePath<Label> >& labelledImagePaths)
  {
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
    int labelledImagePathCount = static_cast<int>(labelledImagePaths.size());
    std::vector<Example_CPtr> result(labelledImagePathCount);

#ifdef WITH_OPENMP
    //#pragma omp parallel for FIXME: check why does not work in multiple threads.
#endif
    for(int i = 0; i < labelledImagePathCount; ++i)
    {
        af::array img = af::loadImage(labelledImagePaths[i].m_imagePath.c_str());
        rafl::Descriptor_CPtr descriptor = spaint::TouchUtil::extract_touch_feature(img);
        result[i].reset(new Example<Label>(descriptor, labelledImagePaths[i].m_label));
#if 0
        std::cout << "Filename: " << labelledImagePaths[i].m_imagePath << " Label: " << labelledImagePaths[i].m_label << std::endl;
#endif
    }

    return result;
  }
};

#endif
