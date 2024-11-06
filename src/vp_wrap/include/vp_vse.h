/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 15:57:35
 ***************************************************************************/
#ifndef VP_VSE_H_
#define VP_VSE_H_

#include "vp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_FRAME_NUM 8u
#define MAX_SHADOW_NUM 8u

#define MAX_BL_NUM 5u

#define PYM_MIN_HEIGHT 32u
#define PYM_MIN_WIDTH 32u
#define PYM_MAX_HEIGHT 4096u
#define PYM_MAX_WIDTH 4096u
#define PYM_SUFFIX_HB_MAX 152u
#define PYM_SUFFIX_HB_MIN 16u
#define PYM_PREFIX_HB_MAX 2u
#define PYM_PREFIX_HB_MIN 0u
#define PYM_SUFFIX_VB_MAX 20u
#define PYM_SUFFIX_VB_MIN 0u
#define PYM_PREFIX_VB_MAX 2u
#define PYM_PREFIX_VB_MIN 0u

#define DEF_PIX_NUM_BF_SOL 2
#define DEF_SUFFIX_HB 100
#define DEF_PREFIX_HB 2
#define DEF_SUFFIX_VB 10
#define DEF_PREFIX_VB 0
#define DEF_BL_MAX_EN 5

#define PYM_STEP_PREC 16u
#define PYM_STEP_PREC_SIZE 65535
#define PYM_SIZ_DIVIDE_2 2u
#define PYM_SIZ_DIVIDE_16 16u

#define PYM_DS_ROI_CFG_SEL_MAX 2u
#define PRE_INT_0_SET 0
#define PRE_INT_1_SET 1
#define PRE_INT_2_SET 2
#define PRE_INT_3_SET 3
#define PRE_INT_4_SET 4
#define PRE_INT_5_SET 5
#define PRE_INT_6_SET 6
#define PRE_INT_7_SET 7
#define PYM4_HW_ID 4u
#define PYM_MAX_HW_ID 6u

#define PYM_MAX_TIMEOUT 10000u
#define PYM_MAX_BUF_NUM 64u
#define PYM_MAX_ADJUST_SRC_BUFFER_NUM 16u

int32_t vp_vse_init(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_vse_start(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_vse_stop(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_vse_deinit(vp_vflow_contex_t *vp_vflow_contex);

int32_t vp_vse_send_frame(vp_vflow_contex_t *vp_vflow_contex,
	hbn_vnode_image_t *image_frame);
int32_t vp_vse_get_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, ImageFrame *frame);
int32_t vp_vse_release_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, ImageFrame *frame);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_VSE_H_
