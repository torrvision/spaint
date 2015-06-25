/**
 * rafl: TreeChopper.cpp
 */

#include "choppers/TreeChopper.h"

namespace rafl {

//#################### CONSTRUCTORS ####################

TreeChopper::TreeChopper(size_t treeCount, size_t period)
: m_period(period), m_time(0), m_treeCount(treeCount)
{}

}
