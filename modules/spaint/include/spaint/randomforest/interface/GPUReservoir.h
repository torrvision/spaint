/**
 * spaint: GPUReservoir.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPURESERVOIR
#define H_SPAINT_GPURESERVOIR

#include "ITMLib/Utils/ITMMath.h"

#include "../../features/interface/RGBDPatchFeature.h"
#include "GPUForestTypes.h"

namespace spaint
{
// Ugly, need to put it somewhere better
struct PositionColourExample
{
  Vector3f position;
  Vector3u colour;
};

class GPUReservoir
{
public:
  typedef PositionColourExample ExampleType;
  typedef ORUtils::Image<ExampleType> ExampleReservoirs;
  typedef boost::shared_ptr<ExampleReservoirs> ExampleReservoirs_Ptr;
  typedef boost::shared_ptr<const ExampleReservoirs> ExampleReservoirs_CPtr;

  GPUReservoir(size_t capacity, size_t nbLeaves, uint32_t rngSeed = 42);
  virtual ~GPUReservoir();

  const ExampleReservoirs_CPtr get_reservoirs() const;

  const ITMIntImage_CPtr get_reservoirs_size() const;

  int get_reservoirs_count() const;

  int get_capacity() const;

  virtual void add_examples(const RGBDPatchFeatureImage_CPtr &features,
      const LeafIndicesImage_CPtr &leafIndices) = 0;
  virtual void clear() = 0;

protected:
  ExampleReservoirs_Ptr m_data;
  ITMIntImage_Ptr m_reservoirsSize;
  ITMIntImage_Ptr m_reservoirsAddCalls;

  uint32_t m_reservoirCapacity;
  uint32_t m_rngSeed;
};

typedef GPUReservoir PositionReservoir;
typedef boost::shared_ptr<PositionReservoir> PositionReservoir_Ptr;
typedef boost::shared_ptr<const PositionReservoir> PositionReservoir_CPtr;

}

#endif
