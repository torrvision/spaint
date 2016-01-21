/**
 * spaintgui: SubwindowConfiguration.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "SubwindowConfiguration.h"

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SubwindowConfiguration::add_subwindow(const Subwindow& subwindow)
{
  m_subwindows.push_back(subwindow);
}

boost::optional<Vector2f> SubwindowConfiguration::compute_fractional_subwindow_position(float viewportFracX, float viewportFracY) const
{
  boost::optional<size_t> subwindowIndex = determine_subwindow_index(viewportFracX, viewportFracY);
  if(!subwindowIndex) return boost::none;

  const Subwindow& subwindow = m_subwindows[*subwindowIndex];
  const Vector2f& tl = subwindow.m_topLeft;
  const Vector2f& br = subwindow.m_bottomRight;

  return Vector2f(
    CLAMP((viewportFracX - tl.x) / (br.x - tl.x), 0.0f, 1.0f),
    CLAMP((viewportFracY - tl.y) / (br.y - tl.y), 0.0f, 1.0f)
  );
}

boost::optional<size_t> SubwindowConfiguration::determine_subwindow_index(float viewportFracX, float viewportFracY) const
{
  for(size_t i = 0, count = m_subwindows.size(); i < count; ++i)
  {
    const Subwindow& subwindow = m_subwindows[i];
    const Vector2f& tl = subwindow.m_topLeft;
    const Vector2f& br = subwindow.m_bottomRight;
    if(tl.x <= viewportFracX && viewportFracX <= br.x &&
       tl.y <= viewportFracY && viewportFracY <= br.y)
    {
      return i;
    }
  }

  return boost::none;
}

size_t SubwindowConfiguration::subwindow_count() const
{
  return m_subwindows.size();
}
