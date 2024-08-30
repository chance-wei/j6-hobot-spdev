/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 16:38:23
 ***************************************************************************/

#include <cstdint>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/fb.h>

#include "utils_log.h"
#include "vpp_display.h"

using namespace std;

namespace spdev
{

/// Class VPPDisplay related

int32_t VPPDisplay::OpenDisplay(int32_t width, int32_t height)
{
	int32_t ret = 0;
	int64_t allocFlags = 0;

	hb_mem_module_open();

	vp_display_init(&m_drm_ctx, width, height);

	allocFlags = HB_MEM_USAGE_MAP_INITIALIZED |
					HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
					HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED |
					HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;

	for (int i = 0; i < NUM_BUFFERS; ++i) {
		memset(&m_vo_buffers[i], 0, sizeof(hbn_vnode_image_t));

		// 使用默认构造函数初始化的 hbn_vnode_image_t 对象
		ret = hb_mem_alloc_graph_buf(width, height,
				MEM_PIX_FMT_NV12,
				allocFlags,
				width, height,
				&m_vo_buffers[i].buffer);
		if (ret != 0) {
			SC_LOGE("hb_mem_alloc_graph_buf for buffer[%d] failed error(%d)", i, ret);
			// 释放已分配的缓冲区
			for (int j = 0; j < i; ++j) {
				hb_mem_free_buf(m_vo_buffers[j].buffer.fd[0]);
			}
			return -1;
		}
	}

	memset(&m_draw_buffers, 0, sizeof(hbn_vnode_image_t));
	ret = hb_mem_alloc_graph_buf(width, height,
			MEM_PIX_FMT_ARGB,
			allocFlags,
			width, height,
			&m_draw_buffers.buffer);
	if (ret != 0) {
		SC_LOGE("hb_mem_alloc_graph_buf for m_draw_buffers failed error(%d)", ret);
		// 释放已分配的缓冲区
		for (int j = 0; j < NUM_BUFFERS; ++j) {
			hb_mem_free_buf(m_vo_buffers[j].buffer.fd[0]);
		}
		return -1;
	}

	return ret;
}

int32_t VPPDisplay::Close()
{
	int32_t ret = 0;
	hb_mem_free_buf(m_draw_buffers.buffer.fd[0]);
	for (int i = 0; i < NUM_BUFFERS; ++i) {
		hb_mem_free_buf(m_vo_buffers[i].buffer.fd[0]);
	}
	vp_display_deinit(&m_drm_ctx);
	hb_mem_module_close();

	return ret;
}

int32_t VPPDisplay::SetImageFrame(ImageFrame *frame)
{
	int32_t ret = 0;

	hbn_vnode_image_t& currentBuffer = m_vo_buffers[currentBufferIndex];
	currentBufferIndex = (currentBufferIndex + 1) % NUM_BUFFERS;

	for (int i = 0; i < frame->plane_count; ++i) {
		memcpy(currentBuffer.buffer.virt_addr[i], frame->data[i], frame->data_size[i]);
	}

	ret = vp_display_set_frame(&m_drm_ctx, 0, &currentBuffer);

	return ret;
}

int32_t VPPDisplay::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
{
	// not support
	return 0;
}

void VPPDisplay::ReturnImageFrame(ImageFrame *frame, int32_t chn)
{
	// not support
}

int32_t VPPDisplay::SetGraphRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
	int32_t flush, uint32_t color, int32_t line_width)
{
	int32_t ret = 0;

	x0 = (x0 < (m_width - line_width)) ? ((x0 >= 0) ? x0 : 0) : (m_width - line_width);
	y0 = (y0 < (m_height - line_width)) ? ((y0 >= 0) ? y0 : 0) : (m_height - line_width);
	x1 = (x1 < (m_width - line_width)) ? ((x1 >= 0) ? x1 : 0) : (m_width - line_width);
	y1 = (y1 < (m_height - line_width)) ? ((y1 >= 0) ? y1 : 0) : (m_height - line_width);

	if (flush) {
		memset(m_draw_buffers.buffer.virt_addr[0], 0, m_width * m_height * DISPLAY_ARGB_BYTES);
	}

	vp_display_draw_rect(m_draw_buffers.buffer.virt_addr[0],
		x0, y0, x1, y1, color, 0, m_width, m_height, line_width);

	ret = vp_display_set_frame(&m_drm_ctx, 1, &m_draw_buffers);
	return ret;
}

int32_t VPPDisplay::SetGraphWord(int32_t x, int32_t y, char *str,
	int32_t flush, uint32_t color, int32_t line_width)
{
	int32_t ret = 0;
	int32_t len;
	if (str == NULL) {
		LOGE_print("string was NULL\n");
		return -1;
	}

	if ((x < 0) || (x > m_width) || (y < 0) || (y > m_height) ||
		((line_width * FONT_ONE_ENCODE_WIDTH + y) > m_height)) {
		LOGE_print("parameter error, coordinate (%d, %d) string:%s line_width:%d\n",
			x, y, str, line_width);
		return -1;
	}
	if (((int)strlen(str) * line_width * FONT_ONE_ENCODE_WIDTH + x) > m_width) {
		len = (m_width - x) / (line_width * FONT_ONE_ENCODE_WIDTH);
		str[len] = '\0';
	}

	if (flush) {
		memset(m_draw_buffers.buffer.virt_addr[0], 0, m_width * m_height * DISPLAY_ARGB_BYTES);
	}

	ret = vp_display_draw_word(m_draw_buffers.buffer.virt_addr[0], x, y, str, m_width, color, line_width);

	return ret;
}

}; // namespace spdev
