/**
 * orx: ORMemoryBlockPtrTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ORX_ORMEMORYBLOCKPTRTYPES
#define H_ORX_ORMEMORYBLOCKPTRTYPES

#include <boost/shared_ptr.hpp>

#include <ORUtils/MemoryBlockTypes.h>

typedef boost::shared_ptr<ORUtils::MemoryBlock<bool> > ORBoolMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<float> > ORFloatMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2f> > ORFloat2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > ORFloat3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4f> > ORFloat4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<int> > ORIntMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2i> > ORInt2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3i> > ORInt3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > ORInt4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<short> > ORShortMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector2s> > ORShort2MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > ORShort3MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4s> > ORShort4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<uchar> > ORUCharMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<Vector4u> > ORUChar4MemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<uint> > ORUIntMemoryBlock_Ptr;
typedef boost::shared_ptr<ORUtils::MemoryBlock<ushort> > ORUShortMemoryBlock_Ptr;

typedef boost::shared_ptr<const ORUtils::MemoryBlock<bool> > ORBoolMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<float> > ORFloatMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2f> > ORFloat2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3f> > ORFloat3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4f> > ORFloat4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<int> > ORIntMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2i> > ORInt2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3i> > ORInt3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4i> > ORInt4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<short> > ORShortMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector2s> > ORShort2MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector3s> > ORShort3MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4s> > ORShort4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<uchar> > ORUCharMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<Vector4u> > ORUChar4MemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<uint> > ORUIntMemoryBlock_CPtr;
typedef boost::shared_ptr<const ORUtils::MemoryBlock<ushort> > ORUShortMemoryBlock_CPtr;

#endif
