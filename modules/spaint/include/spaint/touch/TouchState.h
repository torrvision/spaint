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

  /** A flag indicating whether the surface is being touched. */
  bool m_touchingSurface;

  /** A flag indicating whether the touch position is known. */
  bool m_touchPositionKnown;

  //#################### CONSTRUCTORS ####################
public:
  TouchState();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the touch positions.
   *
   * \return  The touch positions.
   */
  TouchPositions_CPtr get_positions() const;

  /**
   * \brief Sets the touch state.
   *
   * \param position_x          The x positions.
   * \param position_y          The y positions.
   * \param touchingSurface     Whether the surface is being touched or not.
   * \param touchPositionKnown  Whether the touch position is known.
   */
  void set_touch_state(TouchPositions_CPtr touchPositions, bool touchingSurface, bool touchPositionKnown);

  /**
   * \brief Gets whether the surface is being touched.
   *
   * \return The touching-surface flag.
   */
  bool touching_surface() const;

  /**
   * \brief Gets whether the touch positin is known.
   *
   * \return The touch-position-known flag.
   */
  bool touch_position_known() const;
};

//#################### TYPEDEFS ####################
typedef boost::shared_ptr<TouchState> TouchState_Ptr;
typedef boost::shared_ptr<const TouchState> TouchState_CPtr;

}

#endif

