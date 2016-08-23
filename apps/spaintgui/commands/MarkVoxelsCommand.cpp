/**
 * spaintgui:MarkVoxelsCommand.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "MarkVoxelsCommand.h"

#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

MarkVoxelsCommand::MarkVoxelsCommand(const std::string& sceneID, const boost::shared_ptr<const ORUtils::MemoryBlock<Vector3s> >& voxelLocationsMB,
                                     SpaintVoxel::PackedLabel label, const Model_Ptr& model)
: Command(get_static_description()),
  m_label(label),
  m_model(model),
  m_oldVoxelLabelsMB(MemoryBlockFactory::instance().make_block<SpaintVoxel::PackedLabel>(voxelLocationsMB->dataSize)),
  m_sceneID(sceneID),
  m_voxelLocationsMB(voxelLocationsMB)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MarkVoxelsCommand::execute() const
{
  m_model->mark_voxels(m_sceneID, m_voxelLocationsMB, m_label, NORMAL_MARKING, m_oldVoxelLabelsMB);
}

void MarkVoxelsCommand::undo() const
{
  m_model->mark_voxels(m_sceneID, m_voxelLocationsMB, m_oldVoxelLabelsMB, FORCED_MARKING);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::string MarkVoxelsCommand::get_static_description()
{
  return "Mark Voxels";
}
