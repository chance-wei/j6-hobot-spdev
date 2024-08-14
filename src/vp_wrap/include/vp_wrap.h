/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 15:28:19
 * @LastEditTime: 2023-03-05 16:31:05
 ***************************************************************************/
#ifndef VP_WRAP_H_
#define VP_WRAP_H_

#include <stddef.h>

#include "vp_common.h"
#include "vp_vflow.h"
#include "vp_vin.h"
#include "vp_isp.h"
#include "vp_vse.h"
#include "vp_osd.h"
#include "vp_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

void vp_print_debug_infos(void);

void vp_normal_buf_info_print(ImageFrame *frame);

int32_t vp_dump_nv12_to_file(char *filename, uint8_t *data_y, uint8_t *data_uv,
		int width, int height);
int32_t vp_dump_1plane_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size);
int32_t vp_dump_yuv_to_file(char *filename, uint8_t *src_buffer, uint32_t size);

int32_t vp_dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1);

void vp_vin_print_hbn_vnode_image_t(const hbn_vnode_image_t *frame);
void vp_codec_print_media_codec_output_buffer_info(ImageFrame *frame);
void fill_image_frame_from_vnode_image(ImageFrame *frame);
void fill_vnode_image_from_image_frame(ImageFrame *frame);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_WRAP_H_
