/*
 * CubeEyeFrameList.h
 *
 *  Created on: 2020. 1. 8.
 *      Author: erato
 */

#ifndef CUBEEYEFRAMELIST_H_
#define CUBEEYEFRAMELIST_H_

#include "CubeEyeFrame.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeFrameList : public CubeEyeList<sptr_frame>
{
protected:
	CubeEyeFrameList() = default;
	virtual ~CubeEyeFrameList() = default;
};


using sptr_frame_list = std::shared_ptr<CubeEyeFrameList>;
using result_frame_list = std::tuple<result, sptr_frame_list>;

END_NAMESPACE

#endif /* CUBEEYEFRAMELIST_H_ */
