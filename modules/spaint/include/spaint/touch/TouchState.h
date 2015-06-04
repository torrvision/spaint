/**
 * spaint: TouchState.h
 */

#ifndef H_SPAINT_TOUCHSTATE
#define H_SPAINT_TOUCHSTATE

#include <vector>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace spaint {

/**
 * brief An instance of this class can be used to model the touch state.
 */
class TouchState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const std::vector<Eigen::Vector2i> > TouchPositions_CPtr;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The touch positions. */
  TouchPositions_CPtr m_touchPositions;

  /** A flag indicating whether or not the surface is currently being touched. */
  bool m_touchingSurface;

  /** A flag indicating whether or not the touch position is currently known. */
  bool m_touchPositionKnown;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a touch state with an empty set of touch positions.
   */
  TouchState();

  /**
   * \brief Constructs a touch state.
   *
   * \param touchPositions      The touch positions.
   * \param touchingSurface     Whether or not the surface is currently being touched.
   * \param touchPositionKnown  Whether or not the touch position is currently known.
   */
  TouchState(const TouchPositions_CPtr& touchPositions, bool touchingSurface, bool touchPositionKnown);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the touch positions.
   *
   * \return  The touch positions.
   */
  TouchPositions_CPtr get_positions() const;

  /**
   * \brief Gets whether or not the surface is currently being touched.
   *
   * \return  true, if the surface is currently being touched, or false otherwise.
   */
  bool touching_surface() const;

  /**
   * \brief Gets whether or not the touch position is currently known.
   *
   * \return true, if the touch position is currently known, or false otherwise.
   */
  bool touch_position_known() const;
};

}

#endif
