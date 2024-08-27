/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 11:20:22
 * @LastEditTime: 2023-03-05 16:27:23
 ***************************************************************************/
/*
 * Horizon Robotics
 *
 * Copyright (C) 2022 Horizon Robotics Inc.
 * All rights reserved.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _VPP_DISPLAY_H__
#define _VPP_DISPLAY_H__

#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <deque>
#include <stddef.h>

#include "vp_wrap.h"
#include "vpp_module.h"
#include "vp_display.h"

using namespace std;

namespace spdev
{

	// Structure to hold rectangle parameters
	struct RectParams {
		int32_t x0;
		int32_t y0;
		int32_t x1;
		int32_t y1;
		int32_t flush;
		uint32_t color;
		int32_t line_width;
	};

	class VPPDisplay : public VPPModule
	{
	public:
		VPPDisplay()
		{
			SetModuleType(VPP_DISPLAY);
			SetModuleTypeString((char *)"Display");
		}

		virtual ~VPPDisplay() = default;

	public:
		int32_t OpenDisplay(int32_t width = 1920, int32_t height = 1080);

		int32_t Close();

		int32_t SetImageFrame(ImageFrame *frame);

		int32_t GetImageFrame(ImageFrame *frame, int32_t chn = 0, const int32_t timeout = 0);

		/**
		 * @brief 释放图像数据（qbuf）
		 * @param [in] frame   保存图像数据的内存地址
		 * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
		 *
		 * @retval 0      成功
		 * @retval -1      失败
		 */
		void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0);

		int32_t SetGraphRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
							 int32_t flush = 0, uint32_t color = 0xffff0000, int32_t line_width = 4);

		int32_t SetGraphWord(int32_t x, int32_t y, char *str,
							 int32_t flush = 0, uint32_t color = 0xffff0000, int32_t line_width = 1);

	private:
		vp_drm_context_t m_drm_ctx;
		static const int NUM_BUFFERS = 3;
		hbn_vnode_image_t m_vo_buffers[NUM_BUFFERS];
		int32_t currentBufferIndex = 0;
		// Queue to store rectangle parameters
		std::deque<RectParams> rectQueue;
		std::mutex queueMutex; // 用于保护队列的互斥锁
	};

}; // namespace spdev

#endif // _VPP_DISPLAY_H__
