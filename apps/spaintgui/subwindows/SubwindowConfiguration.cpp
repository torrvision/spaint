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

boost::optional<Vector2f> SubwindowConfiguration::compute_fractional_subwindow_position(const Vector2f& fractionalViewportPos) const
{
  boost::optional<size_t> subwindowIndex = determine_subwindow_index(fractionalViewportPos);
  if(!subwindowIndex) return boost::none;

  const Subwindow& subwindow = m_subwindows[*subwindowIndex];
  const Vector2f& tl = subwindow.m_topLeft;
  const Vector2f& br = subwindow.m_bottomRight;

  return Vector2f(
    CLAMP((fractionalViewportPos.x - tl.x) / (br.x - tl.x), 0.0f, 1.0f),
    CLAMP((fractionalViewportPos.y - tl.y) / (br.y - tl.y), 0.0f, 1.0f)
  );
}

boost::optional<size_t> SubwindowConfiguration::determine_subwindow_index(const Vector2f& fractionalViewportPos) const
{
  for(size_t i = 0, count = m_subwindows.size(); i < count; ++i)
  {
    const Subwindow& subwindow = m_subwindows[i];
    const Vector2f& tl = subwindow.m_topLeft;
    const Vector2f& br = subwindow.m_bottomRight;
    if(tl.x <= fractionalViewportPos.x && fractionalViewportPos.x <= br.x &&
       tl.y <= fractionalViewportPos.y && fractionalViewportPos.y <= br.y)
    {
      return i;
    }
  }

  return boost::none;
}

Subwindow& SubwindowConfiguration::subwindow(size_t i)
{
  return m_subwindows[i];
}

const Subwindow& SubwindowConfiguration::subwindow(size_t i) const
{
  return m_subwindows[i];
}

size_t SubwindowConfiguration::subwindow_count() const
{
  return m_subwindows.size();
}
