/*
 * CubeEyeCamera.h
 *
 *  Created on: 2020. 1. 7.
 *      Author: erato
 */

#ifndef CUBEEYECAMERA_H_
#define CUBEEYECAMERA_H_

#include "CubeEyeSource.h"
#include "CubeEyeFrame.h"
#include "CubeEyeProperty.h"


BEGIN_NAMESPACE

class CubeEyeSink;
using sptr_sink_list = std::shared_ptr<CubeEyeList<CubeEyeSink*>>;

class _decl_dll CubeEyeCamera
{
public:
	enum State {
		Released,
		Prepared,
		Stopped,
		Running
	};

public:
	struct IntrinsicParameters {
		struct ForcalLength {
			flt32 fx;
			flt32 fy;
		} forcal;

		struct PrincipalPoint {
			flt32 cx;
			flt32 cy;
		} principal;
	};

	struct DistortionCoefficients {
		struct RadialCoefficient {
			flt64 k1;
			flt64 k2;
			flt64 k3;
		} radial;

		struct TangentialCoefficient {
			flt64 p1;
			flt64 p2;
		} tangential;

		flt64 skewCoefficient;
	};

public:
	class PreparedListener
	{
	public:
		virtual void _decl_call onCubeEyeCameraPrepared(const CubeEyeCamera* camera) = 0;

	protected:
		PreparedListener() = default;
		virtual ~PreparedListener() = default;
	};

public:
	virtual CubeEyeCamera::State _decl_call state() const = 0;
	virtual CubeEyeSource* _decl_call source() const = 0;
	virtual result _decl_call intrinsicParameters(IntrinsicParameters& intrinsic) = 0;
	virtual result _decl_call distortionCoefficients(DistortionCoefficients& distortion) = 0;

public:
	virtual result _decl_call prepare() = 0;
	virtual result _decl_call prepareAsync() = 0;
	virtual result _decl_call run(int32s wantedFrame) = 0;
	virtual result _decl_call stop() = 0;
	virtual result _decl_call release() = 0;

public:
	virtual result _decl_call setProperty(const sptr_property& property) = 0;
	virtual result_property _decl_call getProperty(const std::string& key) = 0;

public:
	virtual sptr_sink_list _decl_call sinkList() const = 0;
	virtual bool _decl_call containsSink(const std::string& sinkName) = 0;
	virtual result _decl_call addSink(CubeEyeSink* sink) = 0;
	virtual result _decl_call removeSink(CubeEyeSink* sink) = 0;
	virtual result _decl_call removeSink(const std::string& sinkName) = 0;
	virtual result _decl_call removeAllSinks() = 0;

public:
	virtual result _decl_call addPreparedListener(PreparedListener* listener) = 0;
	virtual result _decl_call removePreparedListener(PreparedListener* listener) = 0;

protected:
	CubeEyeCamera() = default;
	virtual ~CubeEyeCamera() = default;
};


using sptr_camera = std::shared_ptr<CubeEyeCamera>;

_decl_dll const char* _decl_call last_released_date();
_decl_dll const char* _decl_call last_released_version();
_decl_dll sptr_camera _decl_call create_camera(const sptr_source& source);
_decl_dll sptr_camera _decl_call find_camera(const sptr_source& source);
_decl_dll result _decl_call destroy_camera(const sptr_camera& camera);
_decl_dll result _decl_call set_property(const sptr_property& property);
_decl_dll result_property _decl_call get_property(const std::string& key);

END_NAMESPACE

#endif /* CUBEEYECAMERA_H_ */
