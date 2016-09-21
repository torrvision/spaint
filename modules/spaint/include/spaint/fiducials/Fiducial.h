/**
 * spaint: Fiducial.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIAL
#define H_SPAINT_FIDUCIAL

#include <boost/shared_ptr.hpp>

namespace spaint {

/**
 * \brief An instance of this class represents a fiducial (a reference marker in a 3D scene).
 */
class Fiducial
{
  //#################### PRIVATE VARIABLES ####################
private:
  // TODO

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  // TODO
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Fiducial> Fiducial_Ptr;
typedef boost::shared_ptr<const Fiducial> Fiducial_CPtr;

}

#endif
