/**
 * spaint: TouchState.h
 */

#ifndef H_SPAINT_TOUCHSTATE
#define H_SPAINT_TOUCHSTATE

#include <boost/shared_ptr.hpp>

namespace spaint {

/**
 * brief An instance of this class can be used to model the touch state.
 */
class TouchState
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The x touch positions. */
  std::vector<int> m_position_x;

  /** The y touch positions. */
  std::vector<int> m_position_y;

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
   * \brief Gets the x touch positions.
   *
   * \return   The x touch positions.
   */
  const std::vector<int>& position_x() const;

  /**
   * \brief Gets the y touch positions.
   *
   * \return   The y touch positions.
   */
  const std::vector<int>& position_y() const;

  /**
   * \brief Sets the touch state.
   *
   * \param position_x          The x positions.
   * \param position_y          The y positions.
   * \param touchingSurface     Whether the surface is being touched or not.
   * \param touchPositionKnown  Whether the touch position is known.
   */
  void set_touch_state(const std::vector<int>& position_x, const std::vector<int>& position_y, bool touchingSurface, bool touchPositionKnown);

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

