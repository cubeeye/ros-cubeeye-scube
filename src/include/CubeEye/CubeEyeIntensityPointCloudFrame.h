/*
 * CubeEyeIntensityPointCloudFrame.h
 *
 *  Created on: 2020. 1. 6.
 *      Author: erato
 */

#ifndef CUBEEYEINTENSITYPOINTCLOUDFRAME_H_
#define CUBEEYEINTENSITYPOINTCLOUDFRAME_H_

#include "CubeEyePointCloudFrame.h"

BEGIN_NAMESPACE

template <typename T>
class _decl_dll CubeEyeIntensityPointCloudFrame : public CubeEyePointCloudFrame<T>
{
public:
	virtual CubeEyeList<T>* _decl_call frameDataI() const = 0;

protected:
	CubeEyeIntensityPointCloudFrame() = default;
	virtual ~CubeEyeIntensityPointCloudFrame() = default;
};


using sptr_frame_intensity_pointcloud16u = std::shared_ptr<CubeEyeIntensityPointCloudFrame<int16u>>;
using sptr_frame_intensity_pointcloud32f = std::shared_ptr<CubeEyeIntensityPointCloudFrame<flt32>>;
using sptr_frame_intensity_pointcloud64f = std::shared_ptr<CubeEyeIntensityPointCloudFrame<flt64>>;

_decl_dll sptr_frame_intensity_pointcloud16u _decl_call frame_cast_intensity_pointcloud16u(const sptr_frame& frame);
_decl_dll sptr_frame_intensity_pointcloud32f _decl_call frame_cast_intensity_pointcloud32f(const sptr_frame& frame);
_decl_dll sptr_frame_intensity_pointcloud64f _decl_call frame_cast_intensity_pointcloud64f(const sptr_frame& frame);

END_NAMESPACE

#endif /* CUBEEYEINTENSITYPOINTCLOUDFRAME_H_ */
