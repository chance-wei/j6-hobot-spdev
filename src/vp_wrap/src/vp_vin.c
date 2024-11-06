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
#include "vp_vin.h"

int32_t vp_vin_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	vin_attr_t *vin_attr;
	vin_ichn_attr_t *vin_ichn_attr;
	vin_ochn_attr_t *vin_ochn_attr;
	hbn_vnode_handle_t vnode_magic_id;
	hbn_buf_alloc_attr_t alloc_attr;
	camera_config_t *camera_config;
	deserial_config_t *deserial_config;
	uint32_t hw_id;
	//coverity[misra_c_2012_rule_11_5_violation:SUPPRESS], ## violation reason SYSSW_V_11.5.04
	vin_attr = (vin_attr_t *)vp_vflow_contex->sensor_config->vin_attr;
	camera_config = (camera_config_t *)vp_vflow_contex->sensor_config->camera_config;
	deserial_config = (deserial_config_t *)vp_vflow_contex->sensor_config->deserial_config;
	hw_id = vin_attr->vin_node_attr.cim_attr.mipi_rx;
	vin_ochn_attr = &vin_attr->vin_ochn_attr[VIN_MAIN_FRAME];
	vin_ichn_attr = &vin_attr->vin_ichn_attr;
	vin_attr->vin_node_attr.magicNumber = MAGIC_NUMBER;
	vin_ochn_attr->magicNumber = MAGIC_NUMBER;

	// 创建pipeline中的vin node
	ret = hbn_camera_create(camera_config, &vp_vflow_contex->cam_fd);
	SC_ERR_CON_EQ(ret, 0, "hbn_camera_create");

	/* deserial init */
	ret = hbn_deserial_create(deserial_config, &vp_vflow_contex->des_fd);
	SC_ERR_CON_EQ(ret, 0, "hbn_deserial_create failed");

	ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, &vnode_magic_id);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_open fail ret %d\n", ret);
		return ret;
	}
	vp_vflow_contex->vin_node_handle = vnode_magic_id;

	ret = hbn_vnode_set_attr(vnode_magic_id, vin_attr);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_attr fail ret %d\n", ret);
		hbn_vnode_close(vnode_magic_id);
		return ret;
	}

	ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, vin_ichn_attr);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_ichn_attr fail ret %d\n", ret);
		hbn_vnode_close(vnode_magic_id);
		return ret;
	}

	ret = hbn_vnode_set_ochn_attr(vnode_magic_id, (uint32_t)VIN_MAIN_FRAME, vin_ochn_attr);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_ochn_attr fail ret %d\n", ret);
		hbn_vnode_close(vnode_magic_id);
		return ret;
	}

	memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
	alloc_attr.buffers_num = vin_attr->vin_ochn_buff_attr[VIN_MAIN_FRAME].buffers_num;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN | (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);

	if (vin_ochn_attr->ddr_en) {
		ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, (uint32_t)VIN_MAIN_FRAME, &alloc_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_buf_attr fail ret %d\n", ret);
			hbn_vnode_close(vnode_magic_id);
			return ret;
		}
	}

	vin_ochn_attr = &vin_attr->vin_ochn_attr[VIN_ROI];
	vin_ochn_attr->magicNumber = MAGIC_NUMBER;
	if (vin_ochn_attr->roi_en) {
		ret = hbn_vnode_set_ochn_attr(vnode_magic_id, (uint32_t)VIN_ROI, vin_ochn_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_attr VIN_ROI fail ret %d\n", ret);
			hbn_vnode_close(vnode_magic_id);
			return ret;
		}
		ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, (uint32_t)VIN_ROI, &alloc_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_buf_attr VIN_ROI fail ret %d\n", ret);
			hbn_vnode_close(vnode_magic_id);
			return ret;
		}
	}

	vin_ochn_attr = &vin_attr->vin_ochn_attr[VIN_EMB];
	vin_ochn_attr->magicNumber = MAGIC_NUMBER;
	if (vin_ochn_attr->emb_en) {
		ret = hbn_vnode_set_ochn_attr(vnode_magic_id, (uint32_t)VIN_EMB, vin_ochn_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_attr VIN_EMB fail ret %d\n", ret);
			hbn_vnode_close(vnode_magic_id);
			return ret;
		}
		ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, (uint32_t)VIN_EMB, &alloc_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_buf_attr VIN_EMB fail ret %d\n", ret);
			hbn_vnode_close(vnode_magic_id);
			return ret;
		}
	}

	return 0;

	//int32_t ret = 0;
//
	//vp_sensor_config_t *sensor_config = vp_vflow_contex->sensor_config;
	//camera_config_t *camera_config = sensor_config->camera_config;
//
	//vin_node_attr_t *vin_node_attr = sensor_config->vin_node_attr;
	//vin_ichn_attr_t *vin_ichn_attr = sensor_config->vin_ichn_attr;
	//vin_ochn_attr_t *vin_ochn_attr = sensor_config->vin_ochn_attr;
	//// 调整 mipi_rx 的 index
	//vin_node_attr->cim_attr.mipi_rx = vp_vflow_contex->mipi_csi_rx_index;
	//uint32_t hw_id = vin_node_attr->cim_attr.mipi_rx;
	//uint32_t chn_id = 0;
	//uint64_t vin_attr_ex_mask = 0;
	//hbn_vnode_handle_t *vin_node_handle = &vp_vflow_contex->vin_node_handle;
	//vin_attr_ex_t vin_attr_ex;
	//hbn_buf_alloc_attr_t alloc_attr = {0};
//
	//if(vp_vflow_contex->mclk_is_not_configed){
	//	vin_attr_ex.vin_attr_ex_mask = 0x00;
	//	vin_attr_ex.mclk_ex_attr.mclk_freq = 0; // 使用外部有源晶振
	//	SC_LOGI("csi%d ignore mclk ex attr, because mclk is not configed at device tree.",
	//		vp_vflow_contex->mipi_csi_rx_index);
	//}else{
	//	vin_attr_ex.mclk_ex_attr.mclk_freq = 24000000; // 24MHz
	//	vin_attr_ex.vin_attr_ex_mask = 0x80;
	//}
//
	//// 创建pipeline中的vin node
	//ret = hbn_camera_create(camera_config, &vp_vflow_contex->cam_fd);
	//SC_ERR_CON_EQ(ret, 0, "hbn_camera_create");
	//ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, vin_node_handle);
	//SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");
	//// 设置基本属性
	//ret = hbn_vnode_set_attr(*vin_node_handle, vin_node_attr);
	//SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");
	//// 设置输入通道的属性
	//ret = hbn_vnode_set_ichn_attr(*vin_node_handle, chn_id, vin_ichn_attr);
	//SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");
	//// 设置输出通道的属性
	//// 使能DDR输出
	//vin_ochn_attr->ddr_en = 1;
	//ret = hbn_vnode_set_ochn_attr(*vin_node_handle, chn_id, vin_ochn_attr);
	//SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");
	//vin_attr_ex_mask = vin_attr_ex.vin_attr_ex_mask;
	//if (vin_attr_ex_mask) {
	//	for (uint8_t i = 0; i < VIN_ATTR_EX_INVALID; i ++) {
	//		if ((vin_attr_ex_mask & (1 << i)) == 0)
	//			continue;
//
	//		vin_attr_ex.ex_attr_type = i;
	//		/*we need to set hbn_vnode_set_attr_ex in a loop*/
	//		ret = hbn_vnode_set_attr_ex(*vin_node_handle, &vin_attr_ex);
	//		SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr_ex");
	//	}
	//}
	//alloc_attr.buffers_num = 3;
	//alloc_attr.is_contig = 1;
	//alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
	//					| HB_MEM_USAGE_CPU_WRITE_OFTEN
	//					| HB_MEM_USAGE_CACHED
	//					| HB_MEM_USAGE_HW_CIM
	//					| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	//ret = hbn_vnode_set_ochn_buf_attr(*vin_node_handle, chn_id, &alloc_attr);
	//SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
	//SC_LOGD("successful");
	//return 0;
}

int32_t vp_vin_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->vin_node_handle);
	hbn_deserial_destroy(vp_vflow_contex->des_fd);
	return hbn_camera_destroy(vp_vflow_contex->cam_fd);
}

int32_t vp_vin_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vin_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vin_get_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{
	int32_t ret = 0;
	uint32_t chn_id = 0;
	hbn_vnode_handle_t vin_node_handle = vp_vflow_contex->vin_node_handle;

	ret = hbn_vnode_getframe(vin_node_handle, chn_id, VP_GET_FRAME_TIMEOUT, &frame->vnode_image);
	if (ret != 0) {
		SC_LOGE("hbn_vnode_getframe %d CIM failed(%d)\n", chn_id, ret);
	}

	fill_image_frame_from_vnode_image(frame);

	return ret;
}

int32_t vp_vin_release_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{
	int32_t ret = 0;
	uint32_t chn_id = 0;
	hbn_vnode_handle_t vin_node_handle = vp_vflow_contex->vin_node_handle;
	hbn_vnode_image_t *vnode_image = &frame->vnode_image;

	ret = hbn_vnode_releaseframe(vin_node_handle, chn_id, vnode_image);

	return ret;
}
