/*
 * D-Robotics
 * Copyright (C) 2024 D-Robotics Inc.
 */

#ifndef _VPP_PYTHON_H_
#define _VPP_PYTHON_H_

#include <stdio.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <Python.h>

#include "vpp_module.h"

#define __VPP_API__ __attribute__((visibility("default")))

#define EN_DBG 0

#define PRINT(fmt, ...)                                              \
	do {                                                             \
		printf("[%s]:[%d]:" fmt, __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)

#define DBG(fmt, ...)                  \
	do {                               \
		if (EN_DBG) {                  \
			PRINT(fmt, ##__VA_ARGS__); \
		}                              \
	} while (0)

#define ATOMIC_READ_HEAD(fd, buf, count)  pread(fd, buf, count, SEEK_SET)
#define ATOMIC_WRITE_HEAD(fd, buf, count) pwrite(fd, buf, count, SEEK_SET)

#ifdef __cplusplus
extern "C" {
#endif

namespace spdev
{

	typedef struct {
		PyObject_HEAD;
		void *pobj;
		ImageFrame *pframe;
		VPP_Object_e object;
	} libsppydev_Object;

}

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
