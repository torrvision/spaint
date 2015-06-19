/**
 * spaintgui:MarkVoxelsCommand.cpp
 */

#include "MarkVoxelsCommand.h"

#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

MarkVoxelsCommand::MarkVoxelsCommand(const boost::shared_ptr<const ORUtils::MemoryBlock<Vector3s> >& voxelLocationsMB, unsigned char label,
                                     const spaint::SpaintInteractor_Ptr& interactor)
: Command(get_static_description()),
  m_interactor(interactor),
  m_label(label),
  m_oldVoxelLabelsMB(MemoryBlockFactory::instance().make_block<unsigned char>(voxelLocationsMB->dataSize)),
  m_voxelLocationsMB(voxelLocationsMB)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MarkVoxelsCommand::execute() const
{
  m_interactor->mark_voxels(m_voxelLocationsMB, m_label, m_oldVoxelLabelsMB);
}

void MarkVoxelsCommand::undo() const
{
  m_interactor->mark_voxels(m_voxelLocationsMB, m_oldVoxelLabelsMB);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::string MarkVoxelsCommand::get_static_description()
{
  return "Mark Voxels";
}
