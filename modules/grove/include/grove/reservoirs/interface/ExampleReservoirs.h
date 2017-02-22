/**
 * grove: ExampleReservoirs.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRS
#define H_GROVE_EXAMPLERESERVOIRS

#include <ITMLib/Utils/ITMMath.h>

#include <spaint/util/ITMImagePtrTypes.h>
#include <spaint/util/ITMMemoryBlockPtrTypes.h>

//#include <grove/keypoints/Keypoint3DColour.h>

namespace grove
{

template<typename ExampleType, typename IndexType>
class ExampleReservoirs
{
public:
  typedef ORUtils::Image<ExampleType> ExampleImage;
  typedef boost::shared_ptr<ExampleImage> ExampleImage_Ptr;
  typedef boost::shared_ptr<const ExampleImage> ExampleImage_CPtr;

  typedef ORUtils::Image<ExampleType> ReservoirsImage;
  typedef boost::shared_ptr<ReservoirsImage> ReservoirsImage_Ptr;
  typedef boost::shared_ptr<const ReservoirsImage> ReservoirsImage_CPtr;

  typedef ORUtils::Image<IndexType> IndexImage;
  typedef boost::shared_ptr<IndexImage> IndexImage_Ptr;
  typedef boost::shared_ptr<const IndexImage> IndexImage_CPtr;

  ExampleReservoirs(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);
  virtual ~ExampleReservoirs();

  ReservoirsImage_CPtr get_reservoirs() const;

  ITMIntMemoryBlock_CPtr get_reservoirs_size() const;

  int get_reservoirs_count() const;

  int get_capacity() const;

  virtual void add_examples(const ExampleImage_CPtr &examples,
      const IndexImage_CPtr &reservoirIndices) = 0;
  virtual void clear();

protected:
  uint32_t m_capacity;
  ReservoirsImage_Ptr m_data;
  ITMIntMemoryBlock_Ptr m_reservoirsAddCalls;
  ITMIntMemoryBlock_Ptr m_reservoirsSize;
  uint32_t m_rngSeed;
};

//typedef ExampleReservoirs<grove::Keypoint3DColour, grove::Keypoint3DColour, LeafIndices> PositionReservoir;
//typedef boost::shared_ptr<PositionReservoir> PositionReservoir_Ptr;
//typedef boost::shared_ptr<const PositionReservoir> PositionReservoir_CPtr;

}

#endif
