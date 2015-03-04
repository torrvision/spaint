/**
 * spaint: TouchState.h
 */

#ifndef H_SPAINT_TOUCHSTATE
#define H_SPAINT_TOUCHSTATE

namespace spaint {

/**
 * brief An instance of this class can be used to access the touch position, if a valid position exists.
 */
class TouchState
{
private:
  int m_position_x;
  int m_position_y;
  bool m_touchingSurface;
  bool m_touchPositionKnown;

public:
  TouchState()
  : m_position_x(-1), m_position_y(-1), m_touchingSurface(false), m_touchPositionKnown(false)
  {}

  int position_x() const
  {
    return m_position_x;
  }

  int position_y() const
  {
    return m_position_y;
  }

  void set_touch_state(int position_x, int position_y, bool touchingSurface, bool touchPositionKnown)
  {
    m_position_x = position_x;
    m_position_y = position_y;
    m_touchingSurface = touchingSurface;
    m_touchPositionKnown = touchPositionKnown;
  }

  bool touching_surface() const
  {
    return m_touchingSurface;
  }

  bool touch_position_known() const
  {
    return m_touchPositionKnown;
  }
};

}

#endif

