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
  static std::vector<LabelledPath<Label> > generate_labelled_image_paths(const std::string& imagesPath, const std::string& annotationPath)
  {
    std::vector<LabelledPath<Label> > labelledImagePaths;

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
      labelledImagePaths.push_back(LabelledPath<Label>(imagesPath + "/" + imageName, label));
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

#ifdef WITH_OPENMP
    //#pragma omp parallel for FIXME: check why does not work in multiple threads.
#endif
    for(int i = 0; i < labelledImagePathCount; ++i)
    {
        af::array img = af::loadImage(labelledImagePaths[i].path.c_str());
        rafl::Descriptor_CPtr descriptor = spaint::TouchUtil::calculate_histogram_descriptor(img);
        result[i].reset(new rafl::Example<Label>(descriptor, labelledImagePaths[i].label));
#if 0
        std::cout << "Filename: " << labelledImagePaths[i].m_imagePath << " Label: " << labelledImagePaths[i].m_label << std::endl;
#endif
    }

    return result;
  }
};

#endif
