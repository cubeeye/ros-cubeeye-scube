/*
 * CubeEye.h
 *
 *  Created on: 2019. 12. 26.
 *      Author: erato
 */

#ifndef CUBEEYE_H_
#define CUBEEYE_H_

#include <tuple>
#include <vector>
#include <string>
#include <memory>
#include <iterator>

#define BEGIN_NAMESPACE		namespace meere { namespace sensor {
#define END_NAMESPACE		} /*namespace sensor*/ 	} /*namespace meere*/

#if defined(_WIN32) || defined(_WIN64)
#ifdef _BUILD_FOR_DLL_EXPORT
#define _decl_dll	__declspec(dllexport)
#else //_BUILD_FOR_DLL_EXPORT
#define _decl_dll	__declspec(dllimport)
#endif //!_BUILD_FOR_DLL_EXPORT
#define _decl_call	__stdcall
#else //!defined(_WIN32) && !defined(_WIN64)
#define _decl_dll
#define _decl_call
#endif //defined(_WIN32) || defined(_WIN64)

BEGIN_NAMESPACE

using flt32 = float;
using flt64 = double;
using int8s = int8_t;
using int8u = uint8_t;
using int16s = int16_t;
using int16u = uint16_t;
using int32s = int32_t;
using int32u = uint32_t;
using int64s = int64_t;
using int64u = uint64_t;
using bytes = std::vector<int8u>;


typedef enum result
{
	success = 0,
	fail,
	empty,
	overflow,
	not_found,
	not_exist,
	not_ready,
	not_supported,
	not_implemented,
	not_initialized,
	no_such_device,
	invalid_parameter,
	invalid_operation,
	invalid_data_type,
	out_of_memory,
	out_of_resource,
	out_of_range,
	already_exists,
	already_opened,
	already_running,
	already_initialized,
	using_resources,
	timeout,
	unknown
} result;

END_NAMESPACE

#endif /* CUBEEYE_H_ */
