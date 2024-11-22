#include "vp_sensors.h"
#include "vp_vse.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOE_FPS 30
#define RAW10 0x2B
#define RAW12 0x2C

static mipi_config_t sc230ai_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 1,
		.datatype = RAW12,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 810,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 2149,
		.framelenth = 1125 * 2,
		.settle = 22,
		.channel_num = 1,
		.channel_sel = {0},
	},
};

static camera_config_t sc230ai_camera_config = {
		/* 0 */
		.name = "ovx8bstd",
		.addr = 0x11,
		.eeprom_addr = 0x51,
		.serial_addr = 0x41,
		.sensor_mode = 5,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.extra_mode = 3,
		.config_index = 16384,
		.end_flag = CAMERA_CONFIG_END_FLAG,
};

static poc_config_t g_poc_cfg[] = {
	{
		/* 0 */
		.addr = 0xff,
		.end_flag = POC_CONFIG_END_FLAG,
	},
};

static deserial_config_t sc230ai_deserial_config = {
	.name = "max96712",
	.addr = 0x4b,
	.poc_cfg = &g_poc_cfg[0],
	.end_flag = DESERIAL_CONFIG_END_FLAG,
};
//static vin_node_attr_t sc230ai_vin_node_attr = {
//	//.cim_attr = {
//	//	.mipi_rx = 0,
//	//	.vc_index = 0,
//	//	.ipi_channel = 1,
//	//	.cim_isp_flyby = 1,
//	//	.func = {
//	//		.enable_frame_id = 1,
//	//		.set_init_frame_id = 0,
//	//		.hdr_mode = NOT_HDR,
//	//		.time_stamp_en = 0,
//	//	},
//	//},
//	//.lpwm_attr = {
//	//	.enable = 0,
//	//	.lpwm_chn_attr = {
//	//		{	.trigger_source = 0,
//	//			.trigger_mode = 0,
//	//			.period = 33333,
//	//			.offset = 10,
//	//			.duty_time = 100,
//	//			.threshold = 0,
//	//			.adjust_step = 0,
//	//		},
//	//		{	.trigger_source = 0,
//	//			.trigger_mode = 0,
//	//			.period = 33333,
//	//			.offset = 10,
//	//			.duty_time = 100,
//	//			.threshold = 0,
//	//			.adjust_step = 0,
//	//		},
//	//		{	.trigger_source = 0,
//	//			.trigger_mode = 0,
//	//			.period = 33333,
//	//			.offset = 10,
//	//			.duty_time = 100,
//	//			.threshold = 0,
//	//			.adjust_step = 0,
//	//		},
//	//		{	.trigger_source = 0,
//	//			.trigger_mode = 0,
//	//			.period = 33333,
//	//			.offset = 10,
//	//			.duty_time = 100,
//	//			.threshold = 0,
//	//			.adjust_step = 0,
//	//		},
//	//	},
//	//},
//
//	.cim_attr = {
//		.cim_isp_flyby = 0,
//		.cim_pym_flyby = 0,
//		.mipi_en = 1,
//		.mipi_rx = 4,
//		.vc_index = 0,
//		.ipi_channels = 1,
//		.y_uv_swap = 0, //(uint32_t)vpf_get_json_value(p_node_mipi, "y_uv_swap");
//		.func = {
//			.enable_frame_id = 1,
//			.set_init_frame_id = 1,
//			.enable_pattern = 0,
//			.skip_frame = 0,
//			.input_fps = 0,
//			.output_fps = 0,
//			.skip_nums = 0,
//			.hw_extract_m = 0,
//			.hw_extract_n = 0,
//			.lpwm_trig_sel = (int32_t)LPWM_CHN_INVALID,
//		},
//		.rdma_input = {
//			.rdma_en = 0,
//			.stride = 0,
//			.pack_mode = 1,
//			.buff_num = 0,
//		},
//	},
//};

//static vin_attr_ex_t sc230ai_vin_attr_ex = {
//	.cim_static_attr = {
//		.water_level_mark = 0,
//	},
//};

//static vin_ichn_attr_t sc230ai_vin_ichn_attr = {
//	//.width = SENSOR_WIDTH,
//	//.height = SENSOR_HEIGHT,
//	//.format = RAW12,
//	.width =  1920,
//	.height = 1080,
//	.format = 44,
//};

//static vin_ochn_buff_attr_t sc230ai_vin_ochn_buff_attr[VIN_TYPE_INVALID] = {
//	.[VIN_MAIN_FRAME] = { //vin_ochn0_buff_attr
//		.buffers_num = 6,
//	},
//	.[VIN_EMB] = { //vin_ochn3_buff_attr
//		.buffers_num = 6,
//	},
//	.[VIN_ROI] = { //vin_ochn4_buff_attr
//		.buffers_num = 6,
//	},
//};

//static vin_ochn_attr_t sc230ai_vin_ochn_attr[VIN_TYPE_INVALID] = {
//	//.ddr_en = 0,
//	//.ochn_attr_type = VIN_BASIC_ATTR,
//	//.vin_basic_attr = {
//	//	.format = RAW12,
//	//	// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
//	//	// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例RAW12，1920 x 2 = 1920
//	//	.wstride = (SENSOR_WIDTH) * 2,
//	//},
//
//	.[VIN_MAIN_FRAME] = { //vin_ochn0_attr
//		.ddr_en = 1,
//		.vin_basic_attr = {
//			.format = 44,
//			.wstride = 0,
//			.vstride = 0,
//			.pack_mode = 1,
//		},
//		.pingpong_ring = 1,
//		.roi_en = 0,
//		.roi_attr = {
//			.roi_x = 0,
//			.roi_y = 0,
//			.roi_width = 1920,
//			.roi_height = 1080,
//		},
//		.rawds_en = 0,
//		.rawds_attr = {
//			.rawds_mode = 0,
//		},
//	},
//	.[VIN_EMB] = { //vin_ochn3_attr
//		.emb_en = 0,
//		.emb_attr = {
//			.embeded_dependence = 1,
//			.embeded_width = 1920,
//			.embeded_height = 2,
//		},
//		.vin_basic_attr = {
//			.format = 44,
//			.wstride = 0,
//			.vstride = 0,
//			.pack_mode = 1,
//		},
//		.pingpong_ring = 1,
//		.roi_en = 0,
//		.roi_attr = {
//			.roi_x = 0,
//			.roi_y = 0,
//			.roi_width = 1920,
//			.roi_height = 1080,
//		},
//	},
//	.[VIN_ROI] = { //vin_ochn4_attr
//		.roi_en = 0,
//		.roi_attr = {
//			.roi_x = 0,
//			.roi_y = 0,
//			.roi_width = 1080,
//			.roi_height = 720,
//		},
//		.vin_basic_attr = {
//			.format = 44,
//			.wstride = 0,
//			.vstride = 0,
//			.pack_mode = 1,
//		},
//		.pingpong_ring = 1,
//	},
//};

static isp_attr_t sc230ai_isp_attr = {
	.channel = {
		.hw_id = 0,
		.slot_id = 4,
		.ctx_id = -1, //#define AUTO_ALLOC_ID -1
	},
	.work_mode = 0,
	.hdr_mode = 1,
	.size = {
		.width = 1920,
		.height = 1080,
	},
	.frame_rate = 30,
	.sched_mode = 1,
	.algo_state = 1,
	.isp_combine = {
		.isp_channel_mode = 0, //ISP_CHANNEL_MODE_NORMAL
		.bind_channel = {
			.bind_hw_id = 0,
			.bind_slot_id = 0,
		},
	},
	.clear_record = 0, //json和代码中未拿到，设置为0
	.isp_sw_ctrl = {
		.ae_stat_buf_en = 1,
		.awb_stat_buf_en = 1,
		.ae5bin_stat_buf_en = 1,
		.ctx_buf_en = 0,
		.pixel_consistency_en = 0,
	},
//	.input_mode = 1, // 0: online, 1: mcm, 类似offline
//	.sensor_mode= ISP_NORMAL_M,
//	.crop = {
//		.x = 0,
//		.y = 0,
//		.h = SENSOR_HEIGHT,
//		.w = SENSOR_WIDTH,
//	},
};

static isp_ichn_attr_t sc230ai_isp_ichn_attr = {
	.input_crop_cfg = {
		.enable = 0,
		.rect = {
			.x = 0,
			.y = 0,
			.width = 0,
			.height = 0,
		},
	},
	.in_buf_noclean = 1,
	.in_buf_noncached = 0,

//	.width = SENSOR_WIDTH,
//	.height = SENSOR_HEIGHT,
//	.fmt = FRM_FMT_RAW,
//	.bit_width = 10,
};

static isp_ochn_attr_t sc230ai_isp_ochn_attr = {
	.output_crop_cfg = {
		.enable = 0,
		.rect = {
			.x = 0,
			.y = 0,
			.width = 0,
			.height = 0,
		},
	},
	.out_buf_noinvalid = 1,
	.out_buf_noncached = 0,
	.output_raw_level = 0, //ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA
	.stream_output_mode = 0, //convert_isp_stream_output(1),
	.axi_output_mode = 9, //convert_isp_axi_output(0),
	.buf_num = 3,

//	.ddr_en = 1,
//	.fmt = FRM_FMT_NV12,
//	.bit_width = 8,
};

static vin_attr_t sc230ai_vin_attr = {
	.vin_node_attr = {
		.cim_attr = {
			.cim_isp_flyby = 0,
			.cim_pym_flyby = 0,
			.mipi_en = 1,
			.mipi_rx = 4,
			.vc_index = 0,
			.ipi_channels = 1,
			.y_uv_swap = 0, //(uint32_t)vpf_get_json_value(p_node_mipi, "y_uv_swap");
			.func = {
				.enable_frame_id = 1,
				.set_init_frame_id = 1,
				.enable_pattern = 0,
				.skip_frame = 0,
				.input_fps = 0,
				.output_fps = 0,
				.skip_nums = 0,
				.hw_extract_m = 0,
				.hw_extract_n = 0,
				.lpwm_trig_sel = (int32_t)LPWM_CHN_INVALID,
			},
			.rdma_input = {
				.rdma_en = 0,
				.stride = 0,
				.pack_mode = 1,
				.buff_num = 0,
			},
		},
	},
	.vin_attr_ex = {
		.cim_static_attr = {
			.water_level_mark = 0,
		},
	},
	.vin_ochn_attr = {
		[VIN_MAIN_FRAME] = { //vin_ochn0_attr
			.ddr_en = 1,
			.vin_basic_attr = {
				.format = RAW12,
				.wstride = 0,
				.vstride = 0,
				.pack_mode = 1,
			},
			.pingpong_ring = 1,
			.roi_en = 0,
			.roi_attr = {
				.roi_x = 0,
				.roi_y = 0,
				.roi_width = 1920,
				.roi_height = 1080,
			},
			.rawds_en = 0,
			.rawds_attr = {
				.rawds_mode = 0,
			},
		},
		[VIN_EMB] = { //vin_ochn3_attr
			.emb_en = 0,
			.emb_attr = {
				.embeded_dependence = 1,
				.embeded_width = 1920,
				.embeded_height = 2,
			},
			.vin_basic_attr = {
				.format = RAW12,
				.wstride = 0,
				.vstride = 0,
				.pack_mode = 1,
			},
			.pingpong_ring = 1,
			.roi_en = 0,
			.roi_attr = {
				.roi_x = 0,
				.roi_y = 0,
				.roi_width = 1920,
				.roi_height = 1080,
			},
		},
		[VIN_ROI] = { //vin_ochn4_attr
			.roi_en = 0,
			.roi_attr = {
				.roi_x = 0,
				.roi_y = 0,
				.roi_width = 1920,
				.roi_height = 1080,
			},
			.vin_basic_attr = {
				.format = RAW12,
				.wstride = 0,
				.vstride = 0,
				.pack_mode = 1,
			},
			.pingpong_ring = 1,
		},
	},
	.vin_ichn_attr = {
		.width =  1920,
		.height = 1080,
		.format = 44,
	},
	.vin_ochn_buff_attr = {
		[VIN_MAIN_FRAME] = { //vin_ochn0_buff_attr
			.buffers_num = 6,
		},
		[VIN_EMB] = { //vin_ochn3_buff_attr
			.buffers_num = 6,
		},
		[VIN_ROI] = { //vin_ochn4_buff_attr
			.buffers_num = 6,
		},
	},
	.magicNumber = MAGIC_NUMBER,
};



//		pym_config->hw_id = 1;
//		pym_config->pym_mode = 3;
//		pym_config->slot_id = 0;
//		pym_config->pingpong_ring = 0;
//		pym_config->output_buf_num = 6;
//		pym_config->fb_buf_num = 2;
//		pym_config->timeout = 0;
//		pym_config->threshold_time = 0;
//		pym_config->layer_num_trans_next = 0;
//		pym_config->layer_num_share_prev = -1;
//		pym_config->out_buf_noinvalid = 1;
//		pym_config->out_buf_noncached = 0;
//		pym_config->in_buf_noclean = 1;
//		pym_config->in_buf_noncached = 0;
//
//		pym_config->chn_ctrl.pixel_num_before_sol = DEF_PIX_NUM_BF_SOL;
//	    pym_config->chn_ctrl.invalid_head_lines = 0;
//	    pym_config->chn_ctrl.src_in_width = 1920;
//	    pym_config->chn_ctrl.src_in_height = 1080;
//	    pym_config->chn_ctrl.src_in_stride_y = 1920;
//	    pym_config->chn_ctrl.src_in_stride_uv = 1920;
//	    pym_config->chn_ctrl.suffix_hb_val = DEF_SUFFIX_HB;
//	    pym_config->chn_ctrl.prefix_hb_val = DEF_PREFIX_HB;
//	    pym_config->chn_ctrl.suffix_vb_val = DEF_SUFFIX_VB;
//	    pym_config->chn_ctrl.prefix_vb_val = DEF_PREFIX_VB;
//
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_0_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_1_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_2_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_3_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_4_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_5_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_6_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_y[PRE_INT_7_SET] = 0;
//
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_0_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_1_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_2_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_3_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_4_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_5_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_6_SET] = 0;
//	    pym_config->chn_ctrl.pre_int_set_uv[PRE_INT_7_SET] = 0;
//
//	    pym_config->chn_ctrl.ds_roi_en = 1;
//	    pym_config->chn_ctrl.bl_max_layer_en = DEF_BL_MAX_EN;
//	    pym_config->chn_ctrl.ds_roi_uv_bypass = 0;
//
//		pym_config->chn_ctrl.ds_roi_sel[0] = 0;
//		pym_config->chn_ctrl.ds_roi_layer[0] = 0;
//		pym_config->chn_ctrl.ds_roi_info[0].start_left = 0;
//		pym_config->chn_ctrl.ds_roi_info[0].start_top = 0;
//		pym_config->chn_ctrl.ds_roi_info[0].region_width = 1920;
//		pym_config->chn_ctrl.ds_roi_info[0].region_height = 1080;
//		pym_config->chn_ctrl.ds_roi_info[0].wstride_uv = 1920;
//		pym_config->chn_ctrl.ds_roi_info[0].wstride_y = 1920;
//		pym_config->chn_ctrl.ds_roi_info[0].out_width = 1920;
//		pym_config->chn_ctrl.ds_roi_info[0].out_height = 1080;
//		pym_config->chn_ctrl.ds_roi_info[0].vstride = pym_config->chn_ctrl.ds_roi_info[0].out_height;
//		pym_config->magicNumber = MAGIC_NUMBER;

//static pym_cfg_t sc230ai_pym_config = {
//		.hw_id = 1,
//		.pym_mode = 3,
//		.slot_id = 0,
//		.pingpong_ring = 0,
//		.output_buf_num = 6,
//		.fb_buf_num = 2,
//		.timeout = 0,
//		.threshold_time = 0,
//		.layer_num_trans_next = 0,
//		.layer_num_share_prev = -1,
//		.out_buf_noinvalid = 1,
//		.out_buf_noncached = 0,
//		.in_buf_noclean = 1,
//		.in_buf_noncached = 0,
//		.chn_ctrl = {
//			.pixel_num_before_sol = DEF_PIX_NUM_BF_SOL,
//	    	.invalid_head_lines = 0,
//	    	.src_in_width = 1920,
//	    	.src_in_height = 1080,
//	    	.src_in_stride_y = 1920,
//	    	.src_in_stride_uv = 1920,
//	    	.suffix_hb_val = DEF_SUFFIX_HB,
//	    	.prefix_hb_val = DEF_PREFIX_HB,
//	    	.suffix_vb_val = DEF_SUFFIX_VB,
//	    	.prefix_vb_val = DEF_PREFIX_VB,
//	    	.ds_roi_en = 1,
//	    	.bl_max_layer_en = DEF_BL_MAX_EN,
//	    	.ds_roi_uv_bypass = 0,
//			.pre_int_set_y = {
//				[PRE_INT_0_SET] = 0,
//	    		[PRE_INT_1_SET] = 0,
//	    		[PRE_INT_2_SET] = 0,
//	    		[PRE_INT_3_SET] = 0,
//	    		[PRE_INT_4_SET] = 0,
//	    		[PRE_INT_5_SET] = 0,
//	    		[PRE_INT_6_SET] = 0,
//	    		[PRE_INT_7_SET] = 0,
//			},
//			.pre_int_set_uv = {
//	    		[PRE_INT_0_SET] = 0,
//	    		[PRE_INT_1_SET] = 0,
//	    		[PRE_INT_2_SET] = 0,
//	    		[PRE_INT_3_SET] = 0,
//	    		[PRE_INT_4_SET] = 0,
//	    		[PRE_INT_5_SET] = 0,
//	    		[PRE_INT_6_SET] = 0,
//	    		[PRE_INT_7_SET] = 0,
//			},
//			.ds_roi_sel = {
//				[0] = 0,
//			},
//			.ds_roi_layer = {
//				[0] = 0,
//			},
//			.ds_roi_info = {
//				[0] = {
//					.start_left = 0,
//					.start_top = 0,
//					.region_width = 1920,
//					.region_height = 1080,
//					.wstride_uv = 1920,
//					.wstride_y = 1920,
//					.out_width = 1920,
//					.out_height = 1080,
//					.vstride = 1080, //.out_height,
//				},
//			},
//		},
//	.magicNumber = MAGIC_NUMBER,
//};

vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane = {
	.chip_id_reg = 0x3107,
	.chip_id = 0xcb34,
	.sensor_i2c_addr_list = {0x30, 0x32},
	.sensor_name = "sc230ai-30fps",
	.config_file = "linear_1920x1080_raw10_30fps_1lane.c",
	.camera_config = &sc230ai_camera_config,
	.deserial_config = &sc230ai_deserial_config,
	.vin_attr = &sc230ai_vin_attr,
	//.vin_attr = {
	//	.vin_node_attr = &sc230ai_vin_node_attr,
	//	.vin_attr_ex = &sc230ai_vin_attr_ex,
	//	.vin_ochn_attr = sc230ai_vin_ochn_attr,
	//	.vin_ichn_attr = &sc230ai_vin_ichn_attr,
	//	.vin_ochn_buff_attr= sc230ai_vin_ochn_buff_attr,
	//	.magicNumber = MAGIC_NUMBER,
	//},
	//.vin_ichn_attr = &sc230ai_vin_ichn_attr,
	//.vin_node_attr = &sc230ai_vin_node_attr,
	//.vin_ochn_attr = &sc230ai_vin_ochn_attr,
//	.pym_config    = &sc230ai_pym_config,
	.isp_attr      = &sc230ai_isp_attr,
	.isp_ichn_attr = &sc230ai_isp_ichn_attr,
	.isp_ochn_attr = &sc230ai_isp_ochn_attr,
};
