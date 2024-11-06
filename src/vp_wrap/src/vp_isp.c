/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-23 14:01:59
 * @LastEditTime: 2023-03-05 15:57:48
 ***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils/utils_log.h"

#include "vp_wrap.h"
#include "vp_isp.h"

#define ISP_IP_MAX (2u)
#define FIRMWARE_SENSOR_NUMBER (12u)
#define ISP_IP_SENSOR_SLOTS (FIRMWARE_SENSOR_NUMBER) /* value 12: slots of per isp hw, include sensor slots */

int32_t vp_isp_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret;
	//coverity[misra_c_2012_rule_11_5_violation:SUPPRESS], ## violation reason SYSSW_V_11.5_02
	//isp_cfg_t *isp_cfg = (isp_cfg_t*)isp;
    vp_sensor_config_t *sensor_config = NULL;
	sensor_config = vp_vflow_contex->sensor_config;
	uint32_t hw_id = sensor_config->isp_attr->channel.hw_id;
	uint32_t slot_id = sensor_config->isp_attr->channel.slot_id;
	int32_t ctx_id = sensor_config->isp_attr->channel.ctx_id;
	uint32_t buf_num;
	hbn_vnode_handle_t vnode_magic_id;
	hbn_buf_alloc_attr_t alloc_attr = { 0 };
	if ((hw_id > ISP_IP_MAX) || (slot_id >= ISP_IP_SENSOR_SLOTS)) {
		SC_LOGE("hw_id(%d) or slot_id(%d) invaild.\n", hw_id, slot_id);
		return -HBN_STATUS_ISP_INVALID_PARAMETER;
	}
	ret = hbn_vnode_open(HB_ISP, hw_id, ctx_id, &vnode_magic_id);
	if (ret < 0) {
		SC_LOGE("[hw%d][slot%d]HB_ISP open failed ret = %d\n", hw_id, slot_id, ret);
		return -HBN_STATUS_ISP_NODE_UNEXIST;
	}

	vp_vflow_contex->isp_node_handle = vnode_magic_id;

	ret = hbn_vnode_set_attr(vnode_magic_id, sensor_config->isp_attr);
	if (ret < 0) {
		SC_LOGE("[hw%d][slot%d]HB_ISP set_attr failed ret = %d\n", hw_id, slot_id, ret);
		hbn_vnode_close(vnode_magic_id);
		return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}
	ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, sensor_config->isp_ichn_attr);
	if (ret < 0) {
		SC_LOGE("[hw%d][slot%d]HB_ISP set_ichn_attr failed ret = %d\n", hw_id, slot_id, ret);
		hbn_vnode_close(vnode_magic_id);
		return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}
	ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, sensor_config->isp_ochn_attr);
	if (ret < 0) {
		SC_LOGE("[hw%d][slot%d]HB_ISP set_ochn_attr failed ret = %d\n", hw_id, slot_id, ret);
		hbn_vnode_close(vnode_magic_id);
		return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}
	buf_num = sensor_config->isp_ochn_attr->buf_num;
	if (buf_num > 0u) {
		alloc_attr.buffers_num = buf_num;
		alloc_attr.is_contig = 1;
		if (sensor_config->isp_ochn_attr->out_buf_noncached == 0u)
			alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
					     (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);
		else
			alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
						 (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN);
		ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
		if (ret < 0) {
			SC_LOGE("[hw%d][slot%d]HB_ISP set_ochn_buf_attr failed ret = %d\n", hw_id, slot_id, ret);
			hbn_vnode_close(vnode_magic_id);
			return -HBN_STATUS_ISP_INVALID_PARAMETER;
		}
	}
	SC_LOGD("[hw%d][slot%d]done alloc_attr.flags = %ld, cfg size %ld\n", hw_id, slot_id, alloc_attr.flags, sizeof(isp_cfg_t));
	//return vnode_magic_id;
	return 0;

//	int32_t ret = 0;
//	vp_sensor_config_t *sensor_config = NULL;
//	isp_attr_t      *isp_attr = NULL;
//	isp_ichn_attr_t *isp_ichn_attr = NULL;
//	isp_ochn_attr_t *isp_ochn_attr = NULL;
//	hbn_vnode_handle_t *isp_node_handle = NULL;
//	hbn_buf_alloc_attr_t alloc_attr = {0};
//	uint32_t chn_id = 0;
//
//	sensor_config = vp_vflow_contex->sensor_config;
//	isp_attr = sensor_config->isp_attr;
//	isp_ichn_attr = sensor_config->isp_ichn_attr;
//	isp_ochn_attr = sensor_config->isp_ochn_attr;
//	isp_node_handle = &vp_vflow_contex->isp_node_handle;
//
//	isp_attr->input_mode = 2; // offline
//	ret = hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, isp_node_handle);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");
//	ret = hbn_vnode_set_attr(*isp_node_handle, isp_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");
//	ret = hbn_vnode_set_ochn_attr(*isp_node_handle, chn_id, isp_ochn_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");
//	ret = hbn_vnode_set_ichn_attr(*isp_node_handle, chn_id, isp_ichn_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");
//
//	alloc_attr.buffers_num = 3;
//	alloc_attr.is_contig = 1;
//	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
//						| HB_MEM_USAGE_CPU_WRITE_OFTEN
//						| HB_MEM_USAGE_CACHED
//						| HB_MEM_USAGE_HW_ISP
//						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
//	ret = hbn_vnode_set_ochn_buf_attr(*isp_node_handle, chn_id, &alloc_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
//
//	SC_LOGD("successful");
//	return 0;
}

int32_t vp_isp_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->isp_node_handle);
	SC_LOGD("successful");
	return 0;
}

int32_t vp_isp_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_isp_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_isp_get_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{
	int32_t ret = 0;
	uint32_t chn_id = 0;
	hbn_vnode_handle_t isp_node_handle = vp_vflow_contex->isp_node_handle;
	hbn_vnode_image_group_t *out_image_group = &frame->vnode_image_group;

	ret = hbn_vnode_getframe_group(isp_node_handle, chn_id, VP_GET_FRAME_TIMEOUT, out_image_group);

	if (ret != 0) {
		SC_LOGE("hbn_vnode_getframe %d ISP failed\n", chn_id);
	}

	fill_image_frame_from_vnode_image_group(frame);

	return ret;
}

int32_t vp_isp_release_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{
	int32_t ret = 0;
	uint32_t chn_id = 0;
	hbn_vnode_handle_t isp_node_handle = vp_vflow_contex->isp_node_handle;
	hbn_vnode_image_group_t *out_image_group = &frame->vnode_image_group;

	ret = hbn_vnode_releaseframe_group(isp_node_handle, chn_id, out_image_group);

	return ret;
}
