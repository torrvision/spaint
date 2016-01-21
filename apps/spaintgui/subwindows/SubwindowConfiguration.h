/**
 * spaintgui: SubwindowConfiguration.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_SUBWINDOWCONFIGURATION
#define H_SPAINTGUI_SUBWINDOWCONFIGURATION

#include "Subwindow.h"

/**
 * \brief An instance of this class can be used to represent a configuration of sub-windows into which
 *        different types of scene visualisation can be rendered.
 */
class SubwindowConfiguration
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The subwindows in the configuration. */
  std::vector<Subwindow> m_subwindows;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a sub-window to the configuration.
   *
   * \param subwindow The subwindow to add.
   */
  void add_subwindow(const Subwindow& subwindow);

  /**
   * \brief Computes the fractional position of the specified point in the viewport within the sub-window containing it (if any).
   *
   * \param viewportFracX The fractional x coordinate of the point in the viewport (in the range [0,1]).
   * \param viewportFracY The fractional y coordinate of the point in the viewport (in the range [0,1]).
   * \return              The fractional position of the point in its sub-window (if any), or nothing otherwise.
   */
  boost::optional<Vector2f> compute_fractional_subwindow_position(float viewportFracX, float viewportFracY) const;

  /**
   * \brief Determines the index of the sub-window (if any) containing the specified point in the viewport.
   *
   * \param viewportFracX The fractional x coordinate of the point in the viewport (in the range [0,1]).
   * \param viewportFracY The fractional y coordinate of the point in the viewport (in the range [0,1]).
   * \return              The sub-window index (if any) of the point, or nothing otherwise.
   */
  boost::optional<size_t> determine_subwindow_index(float viewportFracX, float viewportFracY) const;

  /**
   * \brief Gets the number of sub-windows in the configuration.
   *
   * \return  The number of sub-windows in the configuration.
   */
  size_t subwindow_count() const;
};

#endif
