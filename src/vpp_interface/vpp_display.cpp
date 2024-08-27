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

	return ret;
}

int32_t VPPDisplay::Close()
{
	int32_t ret = 0;
	for (int i = 0; i < NUM_BUFFERS; ++i) {
		hb_mem_free_buf(m_vo_buffers[i].buffer.fd[0]);
	}
	vp_display_deinit(&m_drm_ctx);

	return ret;
}

int32_t read_yuvv_nv12_file(const char *filename, char *addr0, char *addr1, uint32_t y_size)
{
	if (filename == NULL || addr0 == NULL || y_size == 0) {
		printf("ERR(%s):null param.\n", __func__);
		return -1;
	}

	FILE *Fd = NULL;
	Fd = fopen(filename, "r");
	char *buffer = NULL;

	if (Fd == NULL) {
		printf("ERR(%s):open(%s) fail\n", __func__, filename);
		return -1;
	}

	buffer = (char *)malloc(y_size + y_size / 2);

	if (fread(buffer, 1, y_size, Fd) != y_size) {
		printf("read bin(%s) to addr fail #1\n", filename);
		return -1;
	}

	if (fread(buffer + y_size, 1, y_size / 2, Fd) != y_size / 2) {
		printf("read bin(%s) to addr fail #2\n", filename);
		return -1;
	}

	memcpy(addr0, buffer, y_size);
	memcpy(addr1, buffer + y_size, y_size / 2);

	fflush(Fd);

	if (Fd)
		fclose(Fd);
	if (buffer)
		free(buffer);

	printf("(%s):file read(%s), y-size(%d)\n", __func__, filename, y_size);
	return 0;
}

int read_nv12_image(char *input_image_path, hbn_vnode_image_t *input_image) {
	int64_t alloc_flags = 0;
	int ret = 0;

	memset(input_image, 0, sizeof(hbn_vnode_image_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
				HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
				HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN |
				HB_MEM_USAGE_CACHED |
				HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hb_mem_alloc_graph_buf(1920,
								1080,
								MEM_PIX_FMT_NV12,
								alloc_flags,
								1920,
								1080,
								&input_image->buffer);
	read_yuvv_nv12_file(input_image_path,
					(char *)(input_image->buffer.virt_addr[0]),
					(char *)(input_image->buffer.virt_addr[1]),
					input_image->buffer.size[0]);

	// 设置一个时间戳
	gettimeofday(&input_image->info.tv, NULL);

	return ret;
}

typedef struct {
	unsigned char* data; // Combined pointer for Y and UV planes
	int width;
	int height;
} nv12_image;

void rgb_to_yuv(unsigned int rgb, unsigned char* y, unsigned char* u, unsigned char* v) {
	// Extract the RGB components
	unsigned char r = (rgb >> 16) & 0xFF;
	unsigned char g = (rgb >> 8) & 0xFF;
	unsigned char b = rgb & 0xFF;

	// Convert RGB to YUV
	*y = (unsigned char)(0.299 * r + 0.587 * g + 0.114 * b);
	*u = (unsigned char)(-0.169 * r - 0.331 * g + 0.500 * b + 128);
	*v = (unsigned char)(0.500 * r - 0.460 * g - 0.040 * b + 128);
}

void draw_point(nv12_image* image, int x, int y, unsigned int color) {
	// Check if the coordinates are within the image boundaries
	if (x < 0 || x >= image->width || y < 0 || y >= image->height) return;

	// Convert the RGB color to YUV
	unsigned char yValue, uValue, vValue;
	rgb_to_yuv(color, &yValue, &uValue, &vValue);

	// Set the Y value
	int yIndex = y * image->width + x;
	image->data[yIndex] = yValue;

	int uvIndex = image->height * image->width
		+ (y / 2) * image->width + x / 2 * 2;

	// Set U and V values
	image->data[uvIndex] = uValue;
	image->data[uvIndex + 1] = vValue;
}

// void draw_line(nv12_image* image, int x1, int y1, int x2, int y2, unsigned int color, int line_width) {
// 	int dx = abs(x2 - x1);
// 	int dy = abs(y2 - y1);
// 	int sx = (x1 < x2) ? 1 : -1;
// 	int sy = (y1 < y2) ? 1 : -1;
// 	int err = dx - dy;
// 	int e2;

// 	// Draw the line with the specified width
// 	while (1) {
// 		// Draw a box around the current point to simulate line width
// 		for (int i = -line_width / 2; i <= line_width / 2; ++i) {
// 			for (int j = -line_width / 2; j <= line_width / 2; ++j) {
// 				int px = x1 + i;
// 				int py = y1 + j;
// 				if (px >= 0 && px < image->width && py >= 0 && py < image->height) {
// 					// Draw the point within the line width range
// 					draw_point(image, px, py, (color & 0xFFFFFF)); // Extract RGB part
// 				}
// 			}
// 		}

// 		if (x1 == x2 && y1 == y2) break;
// 		e2 = 2 * err;
// 		if (e2 > -dy) { err -= dy; x1 += sx; }
// 		if (e2 < dx) { err += dx; y1 += sy; }
// 	}
// }

void draw_line(nv12_image* image, int x1, int y1, int x2, int y2, unsigned int color, int line_width) {
	int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
	int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
	int err = dx + dy, e2;

	while (1) {
		draw_point(image, x1, y1, (color & 0xFFFFFF)); // Extract RGB part
		if (x1 == x2 && y1 == y2) break;
		e2 = 2 * err;
		if (e2 >= dy) { err += dy; x1 += sx; }
		if (e2 <= dx) { err += dx; y1 += sy; }
	}
}


void draw_box(nv12_image* image, int x, int y, int w, int h, unsigned int color, int line_width) {
	// Draw the top and bottom edges
	draw_line(image, x, y, x + w - 1, y, color, line_width);
	draw_line(image, x, y + h - 1, x + w - 1, y + h - 1, color, line_width);

	// Draw the left and right edges
	draw_line(image, x, y, x, y + h - 1, color, line_width);
	draw_line(image, x + w - 1, y, x + w - 1, y + h - 1, color, line_width);
}
int32_t VPPDisplay::SetImageFrame(ImageFrame *frame)
{
	int32_t ret = 0;
	nv12_image image;

	hbn_vnode_image_t& currentBuffer = m_vo_buffers[currentBufferIndex];
	currentBufferIndex = (currentBufferIndex + 1) % NUM_BUFFERS;

	for (int i = 0; i < frame->plane_count; ++i) {
		memcpy(currentBuffer.buffer.virt_addr[i], frame->data[i], frame->data_size[i]);
	}

	image.data = currentBuffer.buffer.virt_addr[0];
	image.height = frame->height;
	image.width = frame->width;

	while (!rectQueue.empty()) {
		// Get parameters from the queue
		RectParams params = rectQueue.front();
		rectQueue.pop_front();

		// Draw the rectangle on the image
		draw_box(&image, params.x0, params.y0, params.x1, params.y1,
			params.color, 1);
	}

	// read_nv12_image("/userdata/1920_1080_NV12.yuv", &currentBuffer);
	ret = vp_display_set_frame(&m_drm_ctx, &currentBuffer);

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
	// std::lock_guard<std::mutex> lock(queueMutex); // 自动加锁和解锁
	// RectParams params = { x0, y0, x1, y1, flush, color, line_width };
	// rectQueue.push_back(params);
	return 0;
}

int32_t VPPDisplay::SetGraphWord(int32_t x, int32_t y, char *str,
	int32_t flush, uint32_t color, int32_t line_width)
{
	int32_t s32Ret = 0;

	return s32Ret;
}

}; // namespace spdev
