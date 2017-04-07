/**
 * itmx: ITMMemoryBlockPtrTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ITMMEMORYBLOCKPTRTYPES
#define H_ITMX_ITMMEMORYBLOCKPTRTYPES

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMemoryBlockTypes.h>

typedef boost::shared_ptr<ORUtils::MemoryBlock<bool> > ITMBoolMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<float> > ITMFloatMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2f> > ITMFloat2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > ITMFloat3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4f> > ITMFloat4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<int> > ITMIntMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2i> > ITMInt2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3i> > ITMInt3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > ITMInt4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<short> > ITMShortMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2s> > ITMShort2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > ITMShort3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4s> > ITMShort4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<uchar> > ITMUCharMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4u> > ITMUChar4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<uint> > ITMUIntMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<ushort> > ITMUShortMemoryBlock_Ptr;

typedef boost::shared_ptr<const ORUtils::MemoryBlock<bool> > ITMBoolMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<float> > ITMFloatMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2f> > ITMFloat2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3f> > ITMFloat3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4f> > ITMFloat4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<int> > ITMIntMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2i> > ITMInt2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3i> > ITMInt3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4i> > ITMInt4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<short> > ITMShortMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2s> > ITMShort2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3s> > ITMShort3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4s> > ITMShort4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<uchar> > ITMUCharMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4u> > ITMUChar4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<uint> > ITMUIntMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<ushort> > ITMUShortMemoryBlock_CPtr;

#endif
