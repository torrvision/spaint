/**
 * itmx: ITMImagePtrTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ITMX_ITMIMAGEPTRTYPES
#define H_ITMX_ITMIMAGEPTRTYPES

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

typedef boost::shared_ptr<ORUtils::Image<bool> > ITMBoolImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<float> > ITMFloatImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2f> > ITMFloat2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3f> > ITMFloat3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4f> > ITMFloat4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<int> > ITMIntImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2i> > ITMInt2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3i> > ITMInt3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4i> > ITMInt4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<short> > ITMShortImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2s> > ITMShort2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3s> > ITMShort3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4s> > ITMShort4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<uchar> > ITMUCharImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4u> > ITMUChar4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<uint> > ITMUIntImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<ushort> > ITMUShortImage_Ptr;

typedef boost::shared_ptr<const ORUtils::Image<bool> > ITMBoolImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<float> > ITMFloatImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2f> > ITMFloat2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3f> > ITMFloat3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4f> > ITMFloat4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<int> > ITMIntImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2i> > ITMInt2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3i> > ITMInt3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4i> > ITMInt4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<short> > ITMShortImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2s> > ITMShort2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3s> > ITMShort3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4s> > ITMShort4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<uchar> > ITMUCharImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4u> > ITMUChar4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<uint> > ITMUIntImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<ushort> > ITMUShortImage_CPtr;

#endif
