/**
 * evaluation: ParamSet.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PARAMSET
#define H_EVALUATION_PARAMSET

#include <iosfwd>
#include <map>
#include <string>

#include <boost/assign/list_of.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/export.hpp>

namespace evaluation {

typedef std::map<std::string,std::string> ParamSet;

#if 0
/**
 * \brief An instance of this class can be used to represent a set of parameters and their associated values.
 */
class ParamSet : public std::map<std::string,std::string>
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an empty parameter set.
   */
  ParamSet() {}

  ParamSet(const std::map<std::string,std::string>& paramSet)
  : std::map<std::string,std::string>(paramSet)
  {}

  /**
   * \brief Constructs a parameter set from a list of (parameter,value) pairs.
   *
   * \param begin An iterator pointing to the start of the list.
   * \param end   An iterator pointing to one past the end of the list.
   */
  template <typename IterType>
  ParamSet(const IterType& begin, const IterType& end)
  : std::map<std::string,std::string>(begin, end)
  {}

  //~~~~~~~~~~~~~~~~~~~~ SERIALIZATION ~~~~~~~~~~~~~~~~~~~~
private:
  /**
   * \brief Serializes the node to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<std::map<std::string,std::string> >(*this);
  }

  friend class boost::serialization::access;
};
#endif

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a parameter set to the specified stream.
 *
 * For example, ["A" -> "1", "B" -> "Foo"] would be output as "A-1_B-Foo".
 *
 * \param os  The stream.
 * \param rhs The parameter set.
 * \return    The stream.
 */
//std::ostream& operator<<(std::ostream& os, const ParamSet& rhs);

/**
 * \brief This struct provides utility functions for manipulating parameter sets.
 */
struct ParamSetUtil
{
  //#################### TYPEDEFS ####################
  typedef std::map<std::string,std::string> ParamSet;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a string representation of a parameter set.
   *
   * For example, ["A" -> "1", "B" -> "Foo"] would become "A-1_B-Foo".
   *
   * \param paramSet  The parameter set.
   * \return          A string representation of the parameter set.
   */
  static std::string param_set_to_string(const ParamSet& paramSet);
};

}

#endif
