/*
 * CubeEyePointCloudFrame.h
 *
 *  Created on: 2020. 1. 6.
 *      Author: erato
 */

#ifndef CUBEEYEPOINTCLOUDFRAME_H_
#define CUBEEYEPOINTCLOUDFRAME_H_

#include "CubeEyeFrame.h"

BEGIN_NAMESPACE

template <typename T>
class _decl_dll CubeEyePointCloudFrame : public CubeEyeFrame
{
public:
	virtual CubeEyeList<T>* _decl_call frameDataX() const = 0;
	virtual CubeEyeList<T>* _decl_call frameDataY() const = 0;
	virtual CubeEyeList<T>* _decl_call frameDataZ() const = 0;

protected:
	CubeEyePointCloudFrame() = default;
	virtual ~CubeEyePointCloudFrame() = default;
};


using sptr_frame_pointcloud16u = std::shared_ptr<CubeEyePointCloudFrame<int16u>>;
using sptr_frame_pointcloud32f = std::shared_ptr<CubeEyePointCloudFrame<flt32>>;
using sptr_frame_pointcloud64f = std::shared_ptr<CubeEyePointCloudFrame<flt64>>;

_decl_dll sptr_frame_pointcloud16u _decl_call frame_cast_pointcloud16u(const sptr_frame& frame);
_decl_dll sptr_frame_pointcloud32f _decl_call frame_cast_pointcloud32f(const sptr_frame& frame);
_decl_dll sptr_frame_pointcloud64f _decl_call frame_cast_pointcloud64f(const sptr_frame& frame);

END_NAMESPACE

#endif /* CUBEEYEPOINTCLOUDFRAME_H_ */
