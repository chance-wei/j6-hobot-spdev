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
#include "vp_vse.h"

static pym_cfg_t pym_common_config = {
		.hw_id = 1,
		.pym_mode = 3,
		.slot_id = 0,
		.pingpong_ring = 0,
		.output_buf_num = 6,
		.fb_buf_num = 2,
		.timeout = 0,
		.threshold_time = 0,
		.layer_num_trans_next = 0,
		.layer_num_share_prev = -1,
		.out_buf_noinvalid = 1,
		.out_buf_noncached = 0,
		.in_buf_noclean = 1,
		.in_buf_noncached = 0,
		.chn_ctrl = {
			.pixel_num_before_sol = DEF_PIX_NUM_BF_SOL,
	    	.invalid_head_lines = 0,
	    	.src_in_width = 1920,
	    	.src_in_height = 1080,
	    	.src_in_stride_y = 1920,
	    	.src_in_stride_uv = 1920,
	    	.suffix_hb_val = DEF_SUFFIX_HB,
	    	.prefix_hb_val = DEF_PREFIX_HB,
	    	.suffix_vb_val = DEF_SUFFIX_VB,
	    	.prefix_vb_val = DEF_PREFIX_VB,
	    	.ds_roi_en = 1,
	    	.bl_max_layer_en = DEF_BL_MAX_EN,
	    	.ds_roi_uv_bypass = 0,
			.pre_int_set_y = {
				[PRE_INT_0_SET] = 0,
	    		[PRE_INT_1_SET] = 0,
	    		[PRE_INT_2_SET] = 0,
	    		[PRE_INT_3_SET] = 0,
	    		[PRE_INT_4_SET] = 0,
	    		[PRE_INT_5_SET] = 0,
	    		[PRE_INT_6_SET] = 0,
	    		[PRE_INT_7_SET] = 0,
			},
			.pre_int_set_uv = {
	    		[PRE_INT_0_SET] = 0,
	    		[PRE_INT_1_SET] = 0,
	    		[PRE_INT_2_SET] = 0,
	    		[PRE_INT_3_SET] = 0,
	    		[PRE_INT_4_SET] = 0,
	    		[PRE_INT_5_SET] = 0,
	    		[PRE_INT_6_SET] = 0,
	    		[PRE_INT_7_SET] = 0,
			},
			.ds_roi_sel = {
				[0] = 0,
			},
			.ds_roi_layer = {
				[0] = 0,
			},
			.ds_roi_info = {
				[0] = {
					.start_left = 0,
					.start_top = 0,
					.region_width = 1920,
					.region_height = 1080,
					.wstride_uv = 1920,
					.wstride_y = 1920,
					.out_width = 1920,
					.out_height = 1080,
					.vstride = 1080, //.out_height,
				},
			},
		},
	.magicNumber = MAGIC_NUMBER,
};
pym_cfg_t *get_vp_pym_common_config (void)
{
	return &pym_common_config;
}
int32_t vp_vse_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	pym_cfg_t *pym_cfg;
	hbn_vnode_handle_t vnode_magic_id;
	hbn_buf_alloc_attr_t alloc_attr;
	//coverity[misra_c_2012_rule_11_5_violation:SUPPRESS], ## violation reason SYSSW_V_11.5.04
	pym_cfg = (pym_cfg_t *)vp_vflow_contex->pym_config;
	ret = hbn_vnode_open(HB_PYM, pym_cfg->hw_id, AUTO_ALLOC_ID, &vnode_magic_id);
	if (ret < 0) {
		//coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1.01
		SC_LOGE("bn_vnode_open failed(%d)", ret);
		goto err;
	}
	vp_vflow_contex->vse_node_handle = vnode_magic_id; // to do 不确定这么写对不对

	ret = hbn_vnode_set_attr(vnode_magic_id, pym_cfg);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_attr failed(%d)", ret);
		//coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1.01
		goto err1;
	}

	ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, pym_cfg);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_ichn_attr failed(%d)", ret);
		//coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1.01
		goto err1;
	}

	ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, pym_cfg);
	if (ret < 0) {
		SC_LOGE("hbn_vnode_set_ochn_attr failed(%d)", ret);
		//coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1.01
		goto err1;
	}

	if (pym_cfg->output_buf_num > 0u) {
		memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr.buffers_num = pym_cfg->output_buf_num;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN | (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN);
		if (pym_cfg->out_buf_noncached == 0u) {
			//coverity[misra_c_2012_rule_10_1_violation:SUPPRESS]
			//coverity[misra_c_2012_rule_10_4_violation:SUPPRESS]
			alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;
		}
		ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
		if (ret < 0) {
			SC_LOGE("hbn_vnode_set_ochn_buf_attr failed(%d)", ret);
			//coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1.01
			goto err1;
		}
	}
	SC_LOGD("done cfg size %ld\n", sizeof(pym_cfg_t));

	return 0;
err1:
	hbn_vnode_close(vnode_magic_id);
err:
	return ret;

//	int32_t ret = 0, i = 0;
//	uint32_t hw_id = 0;
//	uint32_t ichn_id = 0;
//	hbn_buf_alloc_attr_t alloc_attr = {0};
//	hbn_vnode_handle_t *vse_node_handle = NULL;
//	vse_config_t *vse_config = NULL;
//
//	vse_node_handle = &vp_vflow_contex->vse_node_handle;
//	vse_config = &vp_vflow_contex->vse_config;
//
//	ret = hbn_vnode_open(HB_VSE, hw_id, AUTO_ALLOC_ID, vse_node_handle);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");
//
//	ret = hbn_vnode_set_attr(*vse_node_handle, &vse_config->vse_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");
//
//	SC_LOGD("VSE: input_width: %d input_height: %d",
//			vse_config->vse_ichn_attr.width, vse_config->vse_ichn_attr.height);
//	ret = hbn_vnode_set_ichn_attr(*vse_node_handle, ichn_id, &vse_config->vse_ichn_attr);
//	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");
//
//	alloc_attr.buffers_num = 3;
//	alloc_attr.is_contig = 1;
//	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
//						| HB_MEM_USAGE_CPU_WRITE_OFTEN
//						| HB_MEM_USAGE_CACHED
//						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
//
//	for (i = 0; i < VSE_MAX_CHN_NUM; i++) {
//		if (vse_config->vse_ochn_attr[i].chn_en) {
//			ret = hbn_vnode_set_ochn_attr(*vse_node_handle, i, &vse_config->vse_ochn_attr[i]);
//			SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");
//			ret = hbn_vnode_set_ochn_buf_attr(*vse_node_handle, i, &alloc_attr);
//			SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
//		}
//	}
//
//	SC_LOGD("successful");
//	return 0;
}

int32_t vp_vse_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->vse_node_handle);
	SC_LOGD("successful");
	return 0;
}

int32_t vp_vse_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vse_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vse_send_frame(vp_vflow_contex_t *vp_vflow_contex, hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	ret = hbn_vnode_sendframe(vp_vflow_contex->vse_node_handle, 0, image_frame);
	if (ret != 0) {
		SC_LOGE("hbn_vnode_sendframe failed(%d)", ret);
		return -1;
	}
	return ret;
}

int32_t vp_vse_get_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, ImageFrame *frame)
{
	int32_t ret = 0;
	hbn_vnode_handle_t vse_node_handle = vp_vflow_contex->vse_node_handle;
	hbn_vnode_image_group_t *out_image_group = &frame->vnode_image_group;

	ret = hbn_vnode_getframe_group(vse_node_handle, 0, VP_GET_FRAME_TIMEOUT, out_image_group);

	if (ret != 0) {
		SC_LOGE("hbn_vnode_getframe from PYM failed\n");
	}

	fill_image_frame_from_vnode_image_group(frame, ochn_id);

	return ret;
}

int32_t vp_vse_release_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, ImageFrame *frame)
{
	int32_t ret = 0;

	hbn_vnode_handle_t vse_node_handle = vp_vflow_contex->vse_node_handle;
	hbn_vnode_image_group_t *out_image_group = &frame->vnode_image_group;

	ret = hbn_vnode_releaseframe_group(vse_node_handle, 0, out_image_group);

	return ret;
}
