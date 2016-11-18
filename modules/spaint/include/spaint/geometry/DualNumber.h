/**
 * spaint: DualNumber.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_DUALNUMBER
#define H_SPAINT_DUALNUMBER

#include <cassert>
#include <cmath>
#include <iostream>

namespace spaint {

/**
 * \brief An instance of an instantiation of this class template represents a dual number
 *        of the form \hat{n} = r + \epsilon d, where \epsilon^2 = 0.
 *
 *        See "Dual Quaternions for Rigid Transformation Blending" by Kavan et al.
 */
template <typename T>
struct DualNumber
{
  //#################### PUBLIC VARIABLES ####################

  /** The real component of the dual number. */
  T r;

  /** The dual component of the dual number. */
  T d;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a dual number with zero components.
   */
  DualNumber()
  : r(T()), d(T())
  {}

  /**
   * \brief Constructs a dual number from a real one.
   *
   * \param r_  The real number.
   */
  DualNumber(T r_)
  : r(r_), d(0)
  {}

  /**
   * \brief Constructs a dual number with the specified components.
   *
   * \param r_  The real component of the dual number.
   * \param d_  The dual component of the dual number.
   */
  DualNumber(T r_, T d_)
  : r(r_), d(d_)
  {}

  //#################### PUBLIC OPERATORS ####################

  /**
   * \brief Adds another dual number to this one.
   *
   * \param rhs The other dual number.
   * \return    This dual number.
   */
  DualNumber<T>& operator+=(const DualNumber<T>& rhs)
  {
    r += rhs.r;
    d += rhs.d;
    return *this;
  }

  /**
   * \brief Subtracts another dual number from this one.
   *
   * \param rhs The other dual number.
   * \return    This dual number.
   */
  DualNumber<T>& operator-=(const DualNumber<T>& rhs)
  {
    r -= rhs.r;
    d -= rhs.d;
    return *this;
  }

  /**
   * \brief Multiplies another dual number with this one.
   *
   * \param rhs The other dual number.
   * \return    This dual number.
   */
  DualNumber<T>& operator*=(const DualNumber<T>& rhs)
  {
    d = r * rhs.d + d * rhs.r;
    r *= rhs.r;
    return *this;
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################

  /**
   * \brief Calculates the conjugate of this dual number.
   *
   * \return  The conjugate of this dual number.
   */
  DualNumber<T> conjugate() const
  {
    return DualNumber<T>(r, -d);
  }

  /**
   * \brief Calculates the inverse of this dual number.
   *
   * \return  The inverse of this dual number.
   */
  DualNumber<T> inverse() const
  {
    assert(!is_pure());
    return DualNumber<T>(1 / r, -d / (r * r));
  }

  /**
   * \brief Determines whether or not this dual number is pure (has a zero real component).
   *
   * \return  true, if this dual number is pure, or false otherwise.
   */
  bool is_pure() const
  {
    return fabs(r) <= 1e-5;
  }

  /**
   * \brief Calculates the square root of this dual number.
   *
   * \return  The square root of this dual number.
   */
  DualNumber<T> sqrt() const
  {
    assert(r >= 0);
    T rootR = ::sqrt(r);
    return DualNumber<T>(rootR, d / (2 * rootR));
  }
};

//#################### NON-MEMBER OPERATORS ####################

/**
 * \brief Checks whether two dual numbers are equal.
 *
 * \param lhs The first dual number.
 * \param rhs The second dual number.
 * \return    true, if the two dual numbers are equal, or false otherwise.
 */
template <typename T>
bool operator==(const DualNumber<T>& lhs, const DualNumber<T>& rhs)
{
  const T TOL = 1e-4f;
  return fabs(lhs.r - rhs.r) <= TOL && fabs(lhs.d - rhs.d) <= TOL;
}

/**
 * \brief Adds two dual numbers together.
 *
 * \param lhs The first operand.
 * \param rhs The second operand.
 * \return    The result of the operation.
 */
template <typename T>
DualNumber<T> operator+(const DualNumber<T>& lhs, const DualNumber<T>& rhs)
{
  DualNumber<T> copy(lhs);
  copy += rhs;
  return copy;
}

/**
 * \brief Subtracts one dual number from another.
 *
 * \param lhs The first operand.
 * \param rhs The second operand.
 * \return    The result of the operation.
 */
template <typename T>
DualNumber<T> operator-(const DualNumber<T>& lhs, const DualNumber<T>& rhs)
{
  DualNumber<T> copy(lhs);
  copy -= rhs;
  return copy;
}

/**
 * \brief Multiplies two dual numbers together.
 *
 * \param lhs The first operand.
 * \param rhs The second operand.
 * \return    The result of the operation.
 */
template <typename T>
DualNumber<T> operator*(const DualNumber<T>& lhs, const DualNumber<T>& rhs)
{
  DualNumber<T> copy(lhs);
  copy *= rhs;
  return copy;
}

/**
 * \brief Calculates the negation of a dual number.
 *
 * \param rhs The dual number.
 * \return    The negation of the dual number.
 */
template <typename T>
DualNumber<T> operator-(const DualNumber<T>& rhs)
{
  return DualNumber<T>(-rhs.r, -rhs.d);
}

//#################### STREAM OPERATORS ####################

template <typename T>
std::ostream& operator<<(std::ostream& os, const DualNumber<T>& rhs)
{
  os << '(' << rhs.r << ',' << rhs.d << ')';
  return os;
}

//#################### TYPEDEFS ####################

typedef DualNumber<double> DualNumberd;
typedef DualNumber<float> DualNumberf;

}

#endif
