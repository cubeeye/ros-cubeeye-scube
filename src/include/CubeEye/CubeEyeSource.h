/*
 * CubeEyeSource.h
 *
 *  Created on: 2019. 12. 26.
 *      Author: erato
 */

#ifndef CUBEEYESOURCE_H_
#define CUBEEYESOURCE_H_

#include "CubeEyeList.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeSource
{
public:
	virtual std::string _decl_call name() const = 0;
	virtual std::string _decl_call serialNumber() const = 0;
	virtual std::string _decl_call uri() const = 0;

public:
	CubeEyeSource() = default;
	virtual ~CubeEyeSource() = default;
};


class _decl_dll CubeEyeSourceListener
{
public:
	virtual void _decl_call onAttachedCubeEyeSource(const CubeEyeSource* source) = 0;
	virtual void _decl_call onDetachedCubeEyeSource(const CubeEyeSource* source) = 0;

protected:
	CubeEyeSourceListener() = default;
	virtual ~CubeEyeSourceListener() = default;
};


using sptr_source = std::shared_ptr<CubeEyeSource>;
using sptr_source_list = std::shared_ptr<CubeEyeList<sptr_source>>;
using result_source = std::tuple<result, sptr_source>;

_decl_dll sptr_source_list _decl_call search_camera_source();
_decl_dll result_source _decl_call make_camera_source(const char* scheme, \
		int vid, int pid, int bus, int addr, int fd, const char* path, const char* serialNumber);
_decl_dll result _decl_call add_source_listener(CubeEyeSourceListener* listener);
_decl_dll result _decl_call remove_source_listener(CubeEyeSourceListener* listener);


END_NAMESPACE

#endif /* CUBEEYESOURCE_H_ */
