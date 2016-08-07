/**
 * tvgutil: NumberSequenceGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_NUMBERSEQUENCEGENERATOR
#define H_TVGUTIL_NUMBERSEQUENCEGENERATOR

#include <stdexcept>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct provides utility functions for generating number sequences.
 */
struct NumberSequenceGenerator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Generates a number sequence in the range [lowerBound, upperBound] with a specified interval between consecutive numbers.
   *
   * \param lowerBound          The lower bound of the range.
   * \param step                The interval between consecutive numbers (must be strictly positive).
   * \param upperBound          The upper bound of the range.
   * \return                    The sequence of numbers.
   * \throws std::runtime_error If the range is invalid or the step size is <= 0.
   */
  template <typename T>
  static std::vector<T> generate_stepped(T lowerBound, T step, T upperBound)
  {
    if(lowerBound > upperBound) throw std::runtime_error("Invalid bounds for number sequence");
    if(step <= 0) throw std::runtime_error("Invalid step size for number sequence");

    std::vector<T> seq;
    for(T element = lowerBound; element <= upperBound; element += step)
    {
      seq.push_back(element);
    }

    return seq;
  }
};

}

#endif
