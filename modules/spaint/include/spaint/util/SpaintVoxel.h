/**
 * spaint: SpaintVoxel.h
 */

#ifndef H_SPAINT_SPAINTVOXEL
#define H_SPAINT_SPAINTVOXEL

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief An instance of this struct represents a voxel in the reconstructed scene.
 */
struct SpaintVoxel
{
  typedef uchar LabelType;

  _CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
  _CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

  static const bool hasColorInformation = true;

  /** Value of the truncated signed distance transformation. */
  short sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;
  /** RGB colour information stored for this voxel. */
  Vector3u clr;
  /** Number of observations that made up @p clr. */
  uchar w_color;
  /** Semantic label. */
  LabelType label;

  _CPU_AND_GPU_CODE_ SpaintVoxel()
  {
    sdf = SDF_initialValue();
    w_depth = 0;
    clr = (uchar)0;
    w_color = 0;
    label = 0;
  }
};

}

#endif
