/*
 * CubeEyeFrame.h
 *
 *  Created on: 2019. 12. 27.
 *      Author: erato
 */

#ifndef CUBEEYEFRAME_H_
#define CUBEEYEFRAME_H_

#include "CubeEyeData.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeFrame
{
public:
	enum FrameType {
		FrameType_Unknown				= 0x000,
		FrameType_Raw					= 0x001,
		FrameType_Depth					= 0x002,
		FrameType_Amplitude				= 0x004,
		FrameType_PointCloud			= 0x008,
		FrameType_IntensityPointCloud	= 0x010,
	};

public:
	virtual int32s _decl_call frameWidth() const = 0;
	virtual int32s _decl_call frameHeight() const = 0;
	virtual CubeEyeFrame::FrameType _decl_call frameType() const = 0;
	virtual CubeEyeData::DataType _decl_call frameDataType() const = 0;
	virtual int64u _decl_call timestamp() const = 0;

protected:
	CubeEyeFrame() = default;
	virtual ~CubeEyeFrame() = default;
};

using sptr_frame = std::shared_ptr<CubeEyeFrame>;
using result_frame = std::tuple<result, sptr_frame>;

END_NAMESPACE

#endif /* CUBEEYEFRAME_H_ */
