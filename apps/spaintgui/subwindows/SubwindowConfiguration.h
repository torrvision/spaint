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

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * Makes a default sub-window configuration with the specified number of sub-windows.
   *
   * \param subwindowCount  The number of sub-windows the configuration should have (must be in the set {1,3}).
   * \param imgSize         The size of image needed to store the scene visualisation for each sub-window.
   * \return                The sub-window configuration, if the sub-window count was valid, or null otherwise.
   */
  static boost::shared_ptr<SubwindowConfiguration> make_default(size_t subwindowCount, const Vector2i& imgSize);

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
   * \param fractionalViewportPos The fractional position of the point in the viewport (with components in the range [0,1]).
   * \return                      The fractional position of the point in its sub-window (if any), or nothing otherwise.
   */
  boost::optional<Vector2f> compute_fractional_subwindow_position(const Vector2f& fractionalViewportPos) const;

  /**
   * \brief Determines the index of the sub-window (if any) containing the specified point in the viewport.
   *
   * \param fractionalViewportPos The fractional position of the point in the viewport (with components in the range [0,1]).
   * \return                      The sub-window index (if any) of the point, or nothing otherwise.
   */
  boost::optional<size_t> determine_subwindow_index(const Vector2f& fractionalViewportPos) const;

  /**
   * \brief Gets the i'th sub-window in the configuration.
   *
   * \return  The i'th sub-window in the configuration.
   */
  Subwindow& subwindow(size_t i);

  /**
   * \brief Gets the i'th sub-window in the configuration.
   *
   * \return  The i'th sub-window in the configuration.
   */
  const Subwindow& subwindow(size_t i) const;

  /**
   * \brief Gets the number of sub-windows in the configuration.
   *
   * \return  The number of sub-windows in the configuration.
   */
  size_t subwindow_count() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SubwindowConfiguration> SubwindowConfiguration_Ptr;
typedef boost::shared_ptr<const SubwindowConfiguration> SubwindowConfiguration_CPtr;

#endif
