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

class TouchTrainUtil
{
public:
  template <typename Label>
  static std::vector<std::pair<std::string, Label> > load_instances(const std::string& trainingSetPath, const std::string& trainingSetFileName)
  {
    std::vector<std::pair<std::string, Label> > instances;

    std::string filename = trainingSetPath + "/" + trainingSetFileName;
    std::ifstream fs(filename.c_str());
    std::string line;
    while(std::getline(fs, line))
    {
      typedef boost::char_separator<char> sep;
      typedef boost::tokenizer<sep> tokenizer;
      tokenizer tok(line.begin(), line.end(), sep(", \r"));
      std::vector<std::string> tokens(tok.begin(), tok.end());

      std::string imageName = tokens[0];
      Label label = boost::lexical_cast<Label>(tokens.back());
      instances.push_back(std::make_pair(trainingSetPath + "/images/" + imageName, label));
    }

    return instances;
  }

  template <typename Label>
  static std::vector<boost::shared_ptr<const Example<Label> > > generate_examples(const std::vector<std::pair<std::string, Label> >& instances)
  {
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
    std::vector<Example_CPtr> result;

    for(size_t i = 0, size = instances.size(); i < size; ++i)
    {
#if 0
      std::cout << "Filename: " << instances[i].first << " Label: " << instances[i].second << std::endl;
#endif

      af::array img = af::loadImage(instances[i].first.c_str());
      rafl::Descriptor_CPtr descriptor = spaint::TouchUtil::extract_touch_feature(img);
      result.push_back(Example_CPtr(new Example<Label>(descriptor, instances[i].second)));
    }

    return result;
  }
};

#endif
