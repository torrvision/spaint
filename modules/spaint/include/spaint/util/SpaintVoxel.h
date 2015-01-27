/**
 * spaint: SpaintVoxel.h
 */

#ifndef H_SPAINT_SPAINTVOXEL
#define H_SPAINT_SPAINTVOXEL

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

#if 1
/**
 * \brief An instance of this struct represents a voxel in the reconstructed scene.
 */
struct SpaintVoxel
{
  _CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
  _CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

  static const bool hasColorInformation = false;

  /** Value of the truncated signed distance transformation. */
  short sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;
  /** Semantic label. */
  uchar label;

  _CPU_AND_GPU_CODE_ SpaintVoxel()
  {
    sdf = SDF_initialValue();
    w_depth = 0;
    label = 2;
  }
};
#else
struct SpaintVoxel
{
  // TEMPORARY

	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTANT(bool) hasColorInformation = true;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ SpaintVoxel()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = (uchar)0;
		w_color = 0;
	}

  // END TEMPORARY
};
#endif

}

#endif
