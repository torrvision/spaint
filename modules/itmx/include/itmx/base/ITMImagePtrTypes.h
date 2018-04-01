/**
 * itmx: ITMImagePtrTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ITMX_ITMIMAGEPTRTYPES
#define H_ITMX_ITMIMAGEPTRTYPES

#include <boost/shared_ptr.hpp>

#include <ORUtils/ImageTypes.h>

typedef boost::shared_ptr<ORUtils::Image<bool> > ORBoolImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<float> > ORFloatImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2f> > ORFloat2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3f> > ORFloat3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4f> > ORFloat4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<int> > ORIntImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2i> > ORInt2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3i> > ORInt3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4i> > ORInt4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<short> > ORShortImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector2s> > ORShort2Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector3s> > ORShort3Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4s> > ORShort4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<uchar> > ORUCharImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<Vector4u> > ORUChar4Image_Ptr;
typedef boost::shared_ptr<ORUtils::Image<uint> > ORUIntImage_Ptr;
typedef boost::shared_ptr<ORUtils::Image<ushort> > ORUShortImage_Ptr;

typedef boost::shared_ptr<const ORUtils::Image<bool> > ORBoolImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<float> > ORFloatImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2f> > ORFloat2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3f> > ORFloat3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4f> > ORFloat4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<int> > ORIntImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2i> > ORInt2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3i> > ORInt3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4i> > ORInt4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<short> > ORShortImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector2s> > ORShort2Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector3s> > ORShort3Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4s> > ORShort4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<uchar> > ORUCharImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<Vector4u> > ORUChar4Image_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<uint> > ORUIntImage_CPtr;
typedef boost::shared_ptr<const ORUtils::Image<ushort> > ORUShortImage_CPtr;

#endif
