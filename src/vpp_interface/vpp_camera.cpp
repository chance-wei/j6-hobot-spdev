/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-20 17:14:01
 * @LastEditTime: 2023-03-05 16:37:47
 ***************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "utils_log.h"

#include "vp_vin.h"
#include "vp_isp.h"
#include "vp_vse.h"
#include "vp_wrap.h"

#include "vpp_camera.h"

namespace spdev
{
	int32_t VPPCamera::SelectVseChn(int *chn_en, int src_width, int src_height,
		int dst_width, int dst_height)
	{
		if (((dst_width <= src_width) && (dst_height <= src_height)) &&
			(!(*chn_en & 1 << VP_VSE_DS0)) &&
			(dst_width <= 4096 && dst_height <= 3076)) {
			return VP_VSE_DS0;
		}
		if ((dst_width <= src_width) || (dst_height <= src_height)) {
			if ((dst_width <= 1920 && dst_height <= 1080) &&
				(!(*chn_en & 1 << VP_VSE_DS1))) {
				return VP_VSE_DS1;
			} else if ((dst_width <= 1920 && dst_height <= 1080) &&
					(!(*chn_en & 1 << VP_VSE_DS2))) {
				return VP_VSE_DS2;
			} else if ((dst_width <= 1280 && dst_height <= 720) &&
					(!(*chn_en & 1 << VP_VSE_DS3))) {
				return VP_VSE_DS3;
			} else if ((dst_width <= 1280 && dst_height <= 720) &&
					(!(*chn_en & 1 << VP_VSE_DS4))) {
				return VP_VSE_DS4;
			}
		}
		if (((dst_width >= src_width) && (dst_height >= src_height)) &&
				(!(*chn_en & 1 << VP_VSE_DS5)) &&
				(dst_width <= 4096 && dst_height <= 3076)) {
			return VP_VSE_DS5;
		}

		return -1;
	}

	int32_t VPPCamera::CamInitParam(vp_vflow_contex_t *vp_vflow_contex,
					const int pipe_id, const int video_index, int fps,
					int chn_num, int *width, int *height,
					vp_sensors_parameters *parameters)
	{
		int i = 0;
		int ret = 0;
		int mipi_width = 0, mipi_height = 0;
		int vse_chn = 0, vse_chn_en = 0;
		camera_config_t *camera_config = NULL;
		isp_ichn_attr_t *isp_ichn_attr = NULL;
		vse_config_t *vse_config = NULL;
		int32_t input_width = 0, input_height = 0;

		memset(vp_vflow_contex, 0, sizeof(vp_vflow_contex_t));

		// 1. Detect Sensor
		// 如果video_index 为 -1 ，那么遍历板子上的camera接口，否则使用指定的接口号
		// TODO: RDK X5 有两个 CAM 接口，分别对应 mipi host0 和 mipi host2,
		// 用户传入参数会是： -1， 0, 1, 下面的代码会做强制转换
		// 当时更好的方式是读取 /etc/board_config.json 文件中关于camera接口的配置匹配不同的应将
		// 每个sensor会支持不同的分辨率，需要根据传入的 parameters 选择 sensor 配置
		switch (video_index)
		{
		case -1:
			vp_vflow_contex->mipi_csi_rx_index = 0;
			vp_vflow_contex->sensor_config = vp_get_sensor_config_by_mipi_host(0);
			if (vp_vflow_contex->sensor_config != NULL)
				break;
			vp_vflow_contex->mipi_csi_rx_index = 2;
			vp_vflow_contex->sensor_config = vp_get_sensor_config_by_mipi_host(2);
			break;
		case 0:
			vp_vflow_contex->mipi_csi_rx_index = 0;
			vp_vflow_contex->sensor_config = vp_get_sensor_config_by_mipi_host(0);
			break;
		case 1:
			vp_vflow_contex->mipi_csi_rx_index = 2;
			vp_vflow_contex->sensor_config = vp_get_sensor_config_by_mipi_host(2);
			break;
		default:
			SC_LOGE("The video_index is not supported, please use in -1, 0, 1, 2, etc");
			break;
		}

		// 2. Setting Vse channel
		vse_config = &vp_vflow_contex->vse_config;
		isp_ichn_attr = vp_vflow_contex->sensor_config->isp_ichn_attr;

		input_width = isp_ichn_attr->width;
		input_height = isp_ichn_attr->height;
		SC_LOGD("VSE: input_width: %d input_height: %d",
			input_width, input_height);

		vse_config->vse_ichn_attr.width = input_width;
		vse_config->vse_ichn_attr.height = input_height;
		vse_config->vse_ichn_attr.fmt = FRM_FMT_NV12;
		vse_config->vse_ichn_attr.bit_width = 8;

		// 设置VSE通道0输出属性
		for (i = 0; i < chn_num; i++) {
			if ((width[i] == 0) && (height[i] == 0)) {//如果高宽为0，那么就开一个和原始高宽一致的通道
				width[i] = input_width;
				height[i] = input_height;
			}
			vse_chn = SelectVseChn(&vse_chn_en, input_width, input_height, width[i], height[i]);
			if (vse_chn < 0) {
				LOGE_print("Invalid size:%dx%d\n", width[i], height[i]);
				return -1;
			}
			vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
			vse_config->vse_ochn_attr[vse_chn].roi.x = 0;
			vse_config->vse_ochn_attr[vse_chn].roi.y = 0;
			vse_config->vse_ochn_attr[vse_chn].roi.w = input_width;
			vse_config->vse_ochn_attr[vse_chn].roi.h = input_height;
			vse_config->vse_ochn_attr[vse_chn].target_w = width[i];
			vse_config->vse_ochn_attr[vse_chn].target_h = height[i];
			vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
			vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			vse_chn_en |= 1 << vse_chn;
			SC_LOGI("Setting VSE channel-%d: input_width:%d, input_height:%d, dst_w:%d, dst_h:%d",
				vse_chn,
				input_width, input_height, width[i], height[i]);
		}

		m_width = input_width;
		m_height = input_height;

		return ret;
	}

	int32_t VPPCamera::CamInitVseParam(vp_vflow_contex_t *vp_vflow_contex,
		const int pipe_id, int chn_num, int proc_mode,
		int src_width, int src_height, int *dst_width, int *dst_height,
		int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
	{
		int32_t i = 0;
		int32_t ret = 0;
		int vse_chn = 0, vse_chn_en = 0;
		vse_config_t *vse_config = NULL;

		memset(vp_vflow_contex, 0, sizeof(vp_vflow_contex_t));

		vse_config = &vp_vflow_contex->vse_config;

		vse_config->vse_ichn_attr.width = src_width;
		vse_config->vse_ichn_attr.height = src_height;
		vse_config->vse_ichn_attr.fmt = FRM_FMT_NV12;
		vse_config->vse_ichn_attr.bit_width = 8;

		SC_LOGD("VSE: input_width: %d input_height: %d",
			src_width, src_height);

		// 设置VSE通道0输出属性
		for (i = 0; i < chn_num; i++) {
			switch (proc_mode)
			{
			case VPS_SCALE_ROTATE_CROP:
			case VPS_SCALE_CROP:
				if ((crop_width[i] == 0) || (crop_height[i] == 0)) {
					crop_width[i] = src_width;
					crop_height[i] = src_height;
				}
			case VPS_SCALE_ROTATE:
			case VPS_SCALE:
				if ((dst_width[i] == 0) && (dst_height[i] == 0)) {
					if ((crop_width[i] != 0) && (crop_height[i] != 0)) {
						dst_width[i] = crop_width[i];
						dst_height[i] = crop_height[i];
					} else {
						dst_width[i] = src_width;
						dst_height[i] = src_height;
					}
				}
				break;
			default:
				break;
			}

			vse_chn = SelectVseChn(&vse_chn_en, src_width, src_height, dst_width[i], dst_height[i]);
			if (vse_chn < 0) {
				LOGE_print("Invalid size:%dx%d\n", dst_width[i], dst_height[i]);
				return -1;
			}
			if (proc_mode >= VPS_SCALE) {
				vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
				vse_config->vse_ochn_attr[vse_chn].roi.x = 0;
				vse_config->vse_ochn_attr[vse_chn].roi.y = 0;
				vse_config->vse_ochn_attr[vse_chn].roi.w = src_width;
				vse_config->vse_ochn_attr[vse_chn].roi.h = src_height;
				vse_config->vse_ochn_attr[vse_chn].target_w = dst_width[i];
				vse_config->vse_ochn_attr[vse_chn].target_h = dst_height[i];
				vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
				vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			} else if ((proc_mode == VPS_SCALE_CROP)) {
				vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
				vse_config->vse_ochn_attr[vse_chn].roi.x = crop_x[i];
				vse_config->vse_ochn_attr[vse_chn].roi.y = crop_y[i];
				vse_config->vse_ochn_attr[vse_chn].roi.w = crop_width[i];
				vse_config->vse_ochn_attr[vse_chn].roi.h = crop_height[i];
				vse_config->vse_ochn_attr[vse_chn].target_w = dst_width[i];
				vse_config->vse_ochn_attr[vse_chn].target_h = dst_height[i];
				vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
				vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			}
			if ((proc_mode == VPS_SCALE_ROTATE)
				|| (proc_mode == VPS_SCALE_ROTATE_CROP)
				|| (proc_mode == VPS_SCALE_ROTATE_CROP)) {
				SC_LOGE("VSE does not support rotating images");
				return -1;
			}
			vse_chn_en |= 1 << vse_chn;
			SC_LOGI("Setting VSE channel-%d: input_width:%d, input_height:%d, dst_w:%d, dst_h:%d",
				vse_chn,
				src_width, src_height, dst_width[i], dst_height[i]);
		}

		return ret;
	}
	int32_t VPPCamera::OpenCamera(const int pipe_id,
								  const int32_t video_index, int fps,
								  int32_t chn_num,
								  int32_t *width, int32_t *height,
								  vp_sensors_parameters *parameters)
	{
		int32_t ret = 0;
		vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;
		// TODO: 根据 video_index 和 chn_num 完成sensor的探测、初始化、开流
		ret = CamInitParam(vp_vflow_contex, pipe_id, video_index, fps, chn_num,
			width, height, parameters);
		if (ret != 0){
			SC_LOGE("CamInitParam failed error(%d)", ret);
			return -1;
		}

		ret = vp_vin_init(vp_vflow_contex);
		ret |= vp_isp_init(vp_vflow_contex);
		ret |= vp_vse_init(vp_vflow_contex);
		ret |= vp_vflow_init(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline init failed error(%d)", ret);
			return -1;
		}

		ret = vp_vin_start(vp_vflow_contex);
		ret |= vp_isp_start(vp_vflow_contex);
		ret |= vp_vse_start(vp_vflow_contex);
		ret |= vp_vflow_start(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline start failed error(%d)", ret);
			return -1;
		}

		vp_print_debug_infos();

		return ret;
	}

	int32_t VPPCamera::OpenVSE(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
		int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
		int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
	{
		int32_t ret = 0;
		int64_t alloc_flags = 0;
		vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;

		m_only_vse = true;

		ret = CamInitVseParam(vp_vflow_contex, pipe_id, chn_num, proc_mode,
			src_width, src_height,
			dst_width, dst_height, crop_x, crop_y, crop_width, crop_height,
			rotate);
		if (ret != 0){
			SC_LOGE("CamInitParam failed error(%d)", ret);
			return -1;
		}

		m_width = src_width;
		m_height = src_height;

		ret = vp_vse_init(vp_vflow_contex);
		ret |= vp_vflow_init(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline init failed error(%d)", ret);
			return -1;
		}

		ret = vp_vse_start(vp_vflow_contex);
		ret |= vp_vflow_start(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline start failed error(%d)", ret);
			return -1;
		}

		memset(&m_vse_input_image, 0, sizeof(hbn_vnode_image_t));
		alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
					HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
					HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED |
					HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
		ret = hb_mem_alloc_graph_buf(m_width, m_height,
									MEM_PIX_FMT_NV12,
									alloc_flags,
									m_width, m_height,
									&m_vse_input_image.buffer);
		if (ret != 0){
			SC_LOGE("hb_mem_alloc_graph_buf failed error(%d)", ret);
			return -1;
		}

		vp_print_debug_infos();
		return ret;
	}

	int32_t VPPCamera::OpenVPS(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
			int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
			int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
	{
		return OpenVSE(pipe_id, chn_num, proc_mode, src_width, src_height,
			dst_width, dst_height, crop_x, crop_y, crop_width, crop_height,
			rotate);
	}

	int32_t VPPCamera::Close(void)
	{
		int32_t ret = 0;
		vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;
		ret = vp_vflow_stop(vp_vflow_contex);
		if (m_only_vse == false) {
			ret |= vp_vin_stop(vp_vflow_contex);
			ret |= vp_isp_stop(vp_vflow_contex);
		}
		ret |= vp_vse_stop(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline stop failed error(%d)", ret);
			return -1;
		}
		ret = vp_vflow_deinit(vp_vflow_contex);
		ret |= vp_vse_deinit(vp_vflow_contex);
		if (m_only_vse == false) {
			ret |= vp_isp_deinit(vp_vflow_contex);
			ret |= vp_vin_deinit(vp_vflow_contex);
		}
		if (ret != 0){
			SC_LOGE("pipeline deinit failed error(%d)", ret);
			return -1;
		}
		return 0;
	}

	int32_t VPPCamera::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
	{
		int32_t ret = 0;

		ret = vp_vse_get_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
		if (ret)
		{
			return -1;
		}
		fill_image_frame_from_vnode_image(frame);

		return ret;
	}

	// 对一路的三个数据处理模块取数据
	int32_t VPPCamera::GetImageFrame(ImageFrame *frame, DevModule module,
									 int32_t width, int32_t height, const int32_t timeout)
	{
		int32_t ret = 0;
		int32_t chn = 0;

		switch (module)
		{
		case SP_DEV_RAW:
			ret = vp_vin_get_frame(&m_vp_vflow_context, &(frame->vnode_image));
			break;
		case SP_DEV_ISP:
			ret = vp_isp_get_frame(&m_vp_vflow_context, &frame->vnode_image);
			break;
		case SP_DEV_VSE:
			chn = GetChnId(width, height);
			if (chn >= 0) {
				ret = vp_vse_get_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
			} else {
				LOGE_print("get chn from %dx%d failed", width, height);
				return -1;
			}
			break;
		default:
			LOGE_print("Error: module not supported!\n");
			return -1;
		}

		fill_image_frame_from_vnode_image(frame);
		// 计算丢帧数和更新上次帧id
		frame->lost_image_num = frame->frame_id - m_last_frame_id - 1;
		m_last_frame_id = frame->frame_id;
		return ret;
	}

	void VPPCamera::ReturnImageFrame(ImageFrame *frame, int32_t chn)
	{
		vp_vse_release_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
		return;
	}

	void VPPCamera::ReturnImageFrame(ImageFrame *frame, DevModule module,
									 int32_t width, int32_t height)
	{
		int32_t chn = 0;

		switch (module)
		{
		case SP_DEV_RAW:
			vp_vin_release_frame(&m_vp_vflow_context, &frame->vnode_image);
			break;
		case SP_DEV_ISP:
			vp_isp_release_frame(&m_vp_vflow_context, &frame->vnode_image);
			break;
		case SP_DEV_VSE:
			chn = GetChnId(width, height);
			if (chn >= 0) {
				vp_vse_release_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
			} else {
				LOGE_print("get chn from %dx%d failed", width, height);
				return;
			}
			break;
		default:
			LOGE_print("Error: module not supported!\n");
		}
	}

	// 对一路的三个数据处理模块传入数据
	int32_t VPPCamera::SetImageFrame(ImageFrame *frame)
	{
		// fill_vnode_image_from_image_frame(frame);
		// vp_normal_buf_info_print(frame);

		for (int i = 0; i < frame->plane_count; ++i) {
			memcpy(m_vse_input_image.buffer.virt_addr[i],
				frame->data[i], frame->data_size[i]);
		}

		return vp_vse_send_frame(&m_vp_vflow_context, &m_vse_input_image);
	}

	int32_t VPPCamera::GetChnId(int32_t width, int32_t height)
	{
		if ((width == 0) || (height == 0))
		{
			width = GetModuleWidth();
			height = GetModuleHeight();
		}
		// VSE 有多个输出通道，根据用户输入的分辨率，找到对应的通道号
		vse_config_t *vse_config = &m_vp_vflow_context.vse_config;
		for (int32_t i = 0; i < VSE_MAX_CHN_NUM; i++)
		{

			if ((vse_config->vse_ochn_attr[i].chn_en == CAM_TRUE)
				&& (vse_config->vse_ochn_attr[i].target_w == width)
				&& (vse_config->vse_ochn_attr[i].target_h == height))
			{
				return i;
			}
		}
		return -1;
	}

	int32_t VPPCamera::GetChnIdForBind(int32_t width, int32_t height)
	{
		if ((width == 0) || (height == 0))
		{
			width = GetModuleWidth();
			height = GetModuleHeight();
		}
		// VSE 有多个输出通道，根据用户输入的分辨率，找到对应的通道号
		vse_config_t *vse_config = &m_vp_vflow_context.vse_config;
		for (int32_t i = 0; i < VSE_MAX_CHN_NUM; i++)
		{

			if ((vse_config->vse_ochn_attr[i].chn_en == CAM_TRUE)
				&& (vse_config->vse_ochn_attr[i].target_w == width)
				&& (vse_config->vse_ochn_attr[i].target_h == height))
			{
				return i;
			}
		}
		return -1;
	}

	void VPPCamera::PutChnIdForUnBind(int32_t chn_id)
	{
		return;
	}

} // namespace spdev
