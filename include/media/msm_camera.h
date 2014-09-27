/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_MSM_CAMERA_H
#define __LINUX_MSM_CAMERA_H

#ifdef MSM_CAMERA_BIONIC
#include <sys/types.h>
#endif
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#ifdef MSM_CAMERA_GCC
#include <time.h>
#else
#include <linux/time.h>
#endif

#ifdef __KERNEL__
#include <linux/ion.h>
#endif
#define MSM_CAM_IOCTL_MAGIC 'm'

#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info *)

#define MSM_CAM_IOCTL_PICT_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PICT_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data *)

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, unsigned *)

#define MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME \
	_IO(MSM_CAM_IOCTL_MAGIC, 23)

#define MSM_CAM_IOCTL_CTRL_COMMAND_2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_AF_CTRL \
	_IOR(MSM_CAM_IOCTL_MAGIC, 25, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_AF_CTRL_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 26, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_CONFIG_VPE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 27, struct msm_camera_vpe_cfg_cmd *)

#define MSM_CAM_IOCTL_AXI_VPE_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 28, struct msm_camera_vpe_cfg_cmd *)

#define MSM_CAM_IOCTL_STROBE_FLASH_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 29, uint32_t *)

#define MSM_CAM_IOCTL_STROBE_FLASH_CHARGE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 30, uint32_t *)

#define MSM_CAM_IOCTL_STROBE_FLASH_RELEASE \
	_IO(MSM_CAM_IOCTL_MAGIC, 31)

#define MSM_CAM_IOCTL_ERROR_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 32, uint32_t *)

#define MSM_CAM_IOCTL_ABORT_CAPTURE \
	_IO(MSM_CAM_IOCTL_MAGIC, 33)

#define MSM_CAM_IOCTL_ENABLE_OUTPUT_IND  \
    _IOW(MSM_CAM_IOCTL_MAGIC, 34, uint32_t *)

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2
#define MSM_CAMERA_LED_INIT 3
#define MSM_CAMERA_LED_RELEASE 4

#define MSM_CAMERA_STROBE_FLASH_NONE 0
#define MSM_CAMERA_STROBE_FLASH_XENON 1

#define MSM_MAX_CAMERA_SENSORS  5
#define MAX_SENSOR_NAME 32

#define MSM_MAX_CAMERA_CONFIGS 2

#define PP_SNAP  0x01
#define PP_RAW_SNAP ((0x01)<<1)
#define PP_PREV  ((0x01)<<2)
#define PP_THUMB ((0x01)<<3)
#define PP_MASK		(PP_SNAP|PP_RAW_SNAP|PP_PREV|PP_THUMB)

#define MSM_CAM_CTRL_CMD_DONE  0
#define MSM_CAM_SENSOR_VFE_CMD 1

/* Should be same as VIDEO_MAX_PLANES in videodev2.h */
#define MAX_PLANES 8

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd {
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
	uint32_t timeout_ms;
	int resp_fd; /* FIXME: to be used by the kernel, pass-through for now */
};

struct msm_cam_evt_msg {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	uint32_t frame_id;
	void *data;
};

struct msm_vpe_evt_msg {
	unsigned short type; /* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len; /* size in, number of bytes out */
	uint32_t frame_id;
	void *data;
};

#define MSM_CAM_RESP_CTRL         0
#define MSM_CAM_RESP_STAT_EVT_MSG 1
#define MSM_CAM_RESP_V4L2         2
#define MSM_CAM_RESP_MAX          3

/* this one is used to send ctrl/status up to config thread */

struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	int resptype;
	int timeout_ms;
	struct msm_ctrl_cmd ctrl_cmd;
	/* struct  vfe_event_t  stats_event; */
	struct msm_cam_evt_msg stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

#define CMD_GENERAL			0
#define CMD_AXI_CFG_OUT1		1
#define CMD_AXI_CFG_SNAP_O1_AND_O2	2
#define CMD_AXI_CFG_OUT2		3
#define CMD_PICT_T_AXI_CFG		4
#define CMD_PICT_M_AXI_CFG		5
#define CMD_RAW_PICT_AXI_CFG		6
#define CMD_STATS_AXI_CFG 7
#define CMD_STATS_AF_AXI_CFG 8
#define CMD_FRAME_BUF_RELEASE 9
#define CMD_PREV_BUF_CFG 10
#define CMD_SNAP_BUF_RELEASE 11
#define CMD_SNAP_BUF_CFG 12
#define CMD_STATS_DISABLE 13
#define CMD_STATS_AEC_AWB_ENABLE 14
#define CMD_STATS_AF_ENABLE 15
#define CMD_STATS_BUF_RELEASE 16
#define CMD_STATS_AF_BUF_RELEASE 17
#define CMD_STATS_ENABLE 18
#define UPDATE_STATS_INVALID 19

#define CMD_STATS_AEC_ENABLE 20
#define CMD_STATS_AWB_ENABLE 21
#define CMD_STATS_AEC_AXI_CFG 22
#define CMD_STATS_AWB_AXI_CFG 23
#define CMD_STATS_RS_AXI_CFG 24
#define CMD_STATS_CS_AXI_CFG 25
#define CMD_STATS_IHIST_AXI_CFG 26
#define CMD_STATS_SKIN_AXI_CFG 27
#define CMD_STATS_AEC_BUF_RELEASE 28
#define CMD_STATS_AWB_BUF_RELEASE 29
#define CMD_STATS_RS_BUF_RELEASE 30
#define CMD_STATS_CS_BUF_RELEASE 31
#define CMD_STATS_IHIST_BUF_RELEASE 32
#define CMD_STATS_SKIN_BUF_RELEASE 33

#define CMD_AXI_CFG_SNAP_GEMINI		34
#define CMD_AXI_CFG_SNAP		35
#define CMD_AXI_CFG_PREVIEW		36
#define CMD_AXI_CFG_VIDEO		37

#define CMD_STATS_IHIST_ENABLE 38
#define CMD_STATS_RS_ENABLE 39
#define CMD_STATS_CS_ENABLE 40
#define CMD_AXI_CFG_O1_AND_O2 41  
#define CMD_AXI_CFG_CONT_RAW_RGB 42
#define CMD_VPE 43
#define CMD_AXI_CFG_VPE 44

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

struct msm_vpe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

#define MAX_CAMERA_ENABLE_NAME_LEN 32
struct camera_enable_cmd {
	char name[MAX_CAMERA_ENABLE_NAME_LEN];
};

#define MSM_PMEM_OUTPUT1		0
#define MSM_PMEM_OUTPUT2		1
#define MSM_PMEM_OUTPUT1_OUTPUT2	2
#define MSM_PMEM_THUMBNAIL		3
#define MSM_PMEM_MAINIMG		4
#define MSM_PMEM_RAW_MAINIMG		5
#define MSM_PMEM_AEC_AWB		6
#define MSM_PMEM_AF			7
#define MSM_PMEM_AEC			8
#define MSM_PMEM_AWB			9
#define MSM_PMEM_RS			10
#define MSM_PMEM_CS			11
#define MSM_PMEM_IHIST			12
#define MSM_PMEM_SKIN			13
#define MSM_PMEM_VIDEO			14
#define MSM_PMEM_PREVIEW		15
#define MSM_PMEM_RGB_STREAM		16 /* extension */
#define MSM_PMEM_VIDEO_VPE		17
#define MSM_PMEM_MAX			18

#define STAT_AEAW			0
#define STAT_AEC			1
#define STAT_AF				2
#define STAT_AWB			3
#define STAT_RS				4
#define STAT_CS				5
#define STAT_IHIST			6
#define STAT_SKIN			7
#define STAT_MAX			8

#define FRAME_PREVIEW_OUTPUT1		0
#define FRAME_PREVIEW_OUTPUT2		1
#define FRAME_SNAPSHOT			2
#define FRAME_THUMBNAIL			3
#define FRAME_RAW_SNAPSHOT		4
#define FRAME_MAX			5

struct msm_pmem_info {
	int type;
	int fd;
	void *vaddr;
	uint32_t offset;
	uint32_t len;
	uint32_t y_off;
	uint32_t cbcr_off;
	uint8_t active;
};

struct outputCfg {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

#define VIDEO_NODE 0
#define MCTL_NODE 1

#define OUTPUT_1	0
#define OUTPUT_2	1
#define OUTPUT_1_AND_2            2   /* snapshot only */
#define CAMIF_TO_AXI_VIA_OUTPUT_2 3
#define OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2 4
#define OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 5
#define OUTPUT_1_AND_3 6
#define CAMIF_TO_OUTPUT_CONTINUOUS_RAW 7 /* extension */
#define LAST_AXI_OUTPUT_MODE_ENUM = CAMIF_TO_OUTPUT_CONTINUOUS_RAW 8  

#define MSM_FRAME_PREV_1	0
#define MSM_FRAME_PREV_2	1
#define MSM_FRAME_ENC		2

#define OUTPUT_TYPE_P    (1<<0)
#define OUTPUT_TYPE_T    (1<<1)
#define OUTPUT_TYPE_S    (1<<2)
#define OUTPUT_TYPE_V    (1<<3)
#define OUTPUT_TYPE_L    (1<<4)
#define OUTPUT_TYPE_ST_L (1<<5)
#define OUTPUT_TYPE_ST_R (1<<6)
#define OUTPUT_TYPE_ST_D (1<<7)

struct fd_roi_info {
	void *info;
	int info_len;
};

struct msm_mem_map_info {
	uint32_t cookie;
	uint32_t length;
	uint32_t mem_type;
};

#define MSM_MEM_MMAP		0
#define MSM_MEM_USERPTR		1
#define MSM_PLANE_MAX		8
#define MSM_PLANE_Y			0
#define MSM_PLANE_UV		1

struct msm_frame {
	uint16_t fmt; /* extension */
	struct timespec ts;
	int path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
	uint32_t error_code;
};

enum msm_st_frame_packing {
	SIDE_BY_SIDE_HALF,
	SIDE_BY_SIDE_FULL,
	TOP_DOWN_HALF,
	TOP_DOWN_FULL,
};

struct msm_st_crop {
	uint32_t in_w;
	uint32_t in_h;
	uint32_t out_w;
	uint32_t out_h;
};

struct msm_st_half {
	uint32_t buf_p0_off;
	uint32_t buf_p1_off;
	uint32_t buf_p0_stride;
	uint32_t buf_p1_stride;
	uint32_t pix_x_off;
	uint32_t pix_y_off;
	struct msm_st_crop stCropInfo;
};

struct msm_st_frame {
	struct msm_frame buf_info;
	int type;
	enum msm_st_frame_packing packing;
	struct msm_st_half L;
	struct msm_st_half R;
	int frame_id;
};

#define MSM_CAMERA_ERR_MASK (0xFFFFFFFF & 1)

struct stats_buff {
	unsigned long buff;
	int fd;
};

struct msm_stats_buf {
	int type;
	unsigned long buffer;
	int fd;
};

#define MSM_V4L2_VID_CAP_TYPE	0
#define MSM_V4L2_STREAM_ON		1
#define MSM_V4L2_STREAM_OFF		2
#define MSM_V4L2_SNAPSHOT		3
#define MSM_V4L2_QUERY_CTRL		4
#define MSM_V4L2_GET_CTRL		5
#define MSM_V4L2_SET_CTRL		6
#define MSM_V4L2_QUERY			7
#define MSM_V4L2_GET_CROP		8
#define MSM_V4L2_SET_CROP		9
#define V4L2_CAMERA_EXIT 10
#define MSM_V4L2_MAX 11
/* extension end */

struct crop_info {
	void *info;
	int len;
};

struct msm_postproc {
	int ftnum;
	struct msm_frame fthumnail;
	int fmnum;
	struct msm_frame fmain;
};

struct msm_snapshot_pp_status {
	void *status;
};

#define CFG_SET_MODE			0
#define CFG_SET_EFFECT			1
#define CFG_START			2
#define CFG_PWR_UP			3
#define CFG_PWR_DOWN			4
#define CFG_WRITE_EXPOSURE_GAIN		5
#define CFG_SET_DEFAULT_FOCUS		6
#define CFG_MOVE_FOCUS			7
#define CFG_REGISTER_TO_REAL_GAIN	8
#define CFG_REAL_TO_REGISTER_GAIN	9
#define CFG_SET_FPS			10
#define CFG_SET_PICT_FPS		11
#define CFG_SET_BRIGHTNESS		12
#define CFG_SET_CONTRAST		13
#define CFG_SET_ZOOM			14
#define CFG_SET_EXPOSURE_MODE		15
#define CFG_SET_WB			16
#define CFG_SET_ANTIBANDING		17
#define CFG_SET_EXP_GAIN		18
#define CFG_SET_PICT_EXP_GAIN		19
#define CFG_SET_LENS_SHADING		20
#define CFG_GET_PICT_FPS		21
#define CFG_GET_PREV_L_PF		22
#define CFG_GET_PREV_P_PL		23
#define CFG_GET_PICT_L_PF		24
#define CFG_GET_PICT_P_PL		25
#define CFG_GET_AF_MAX_STEPS		26
#define CFG_GET_PICT_MAX_EXP_LC		27
#define CFG_SEND_WB_INFO    28
/* extension begin */
#define CFG_SET_DIMENSION		    29
#define CFG_SET_PREVIEW_MODE		30
#define	CFG_SET_TEST_PATTERN		31
#define CFG_GET_INT_INFO		    32
#define CFG_GET_AF_STATUS		    33
#define CFG_GET_EXIF			    34
#define CFG_SET_SCENE			    35
#define CFG_GET_AF_ASSIST_LIGHT		36
#define CFG_SET_SHARPNESS		    37
#define CFG_SET_IMG_QUALITY		    38
#define CFG_SET_EXPOSURE_COMPENSATION	39
#define CFG_SET_ISO                 40
#define CFG_SET_SENSOR_DIMENSION    41
#define CFG_SET_FLASH               42
#define CFG_GPIO_CTRL			43
#define CFG_I2C_WRITE			44
#define CFG_I2C_READ			45
#define CFG_CSI_CTRL			46
#define CFG_ROM_READ			47
/* extension end */
#define CFG_MAX 					48



#define MOVE_NEAR	0
#define MOVE_FAR	1

#define SENSOR_PREVIEW_MODE		0
#define SENSOR_SNAPSHOT_MODE		1
#define SENSOR_RAW_SNAPSHOT_MODE	2
/* extension begin */
#define SENSOR_MOVIE_MODE		3
#define SENSOR_HALF_RELEASE_MODE	4
#define SENSOR_RAW_RGB_STREAM_MODE	5
#define SENSOR_RAW_SNAPSHOT_START	6
#define SENSOR_RAW_RGB_SNAPSHOT_MODE	7
/* extension end */
#define SENSOR_VIDEO_120FPS_MODE	8

#define SENSOR_QTR_SIZE			0
#define SENSOR_FULL_SIZE		1
#define SENSOR_QVGA_SIZE		2
#define SENSOR_INVALID_SIZE		3

#define CAMERA_EFFECT_OFF		0
#define CAMERA_EFFECT_MONO		1
#define CAMERA_EFFECT_NEGATIVE		2
#define CAMERA_EFFECT_SOLARIZE		3
#define CAMERA_EFFECT_PASTEL 4
#define CAMERA_EFFECT_MOSAIC 5
#define CAMERA_EFFECT_RESIZE 6
#define CAMERA_EFFECT_SEPIA 7
#define CAMERA_EFFECT_POSTERIZE 8
#define CAMERA_EFFECT_WHITEBOARD 9
#define CAMERA_EFFECT_BLACKBOARD 10
#define CAMERA_EFFECT_AQUA 11
#define CAMERA_EFFECT_MAX 12

/* extension begin */
#define CAMERA_WBTYPE_AUTO 1
#define CAMERA_WBTYPE_CUSTOM 2
#define CAMERA_WBTYPE_INCANDESCENT 3
#define CAMERA_WBTYPE_FLUORESCENT 4
#define CAMERA_WBTYPE_DAYLIGHT 5
#define CAMERA_WBTYPE_CLOUDY_DAYLIGHT 6
#define CAMERA_WBTYPE_TWILIGHT 7
#define CAMERA_WBTYPE_SHADE 8

#define CAMERA_AUTO_EXPOSURE_FRAME_AVG 0
#define CAMERA_AUTO_EXPOSURE_CENTER_WEIGHTED 1
#define CAMERA_AUTO_EXPOSURE_SPOT_METERING 2
/* extension end */

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
};

struct focus_cfg {
	int32_t steps;
	int dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

/* extension begin */
struct camera_dimension_t
{
	uint16_t picture_width;
	uint16_t picture_height;
	uint16_t display_width;
	uint16_t display_height;
	uint16_t thumbnail_width;
	uint16_t thumbnail_height;
};

struct camera_preview_dimension_t
{
	uint16_t sensor_width;
	uint16_t sensor_height;
};

enum set_test_pattern_t {
   TEST_PATTERN_ON,
   TEST_PATTERN_OFF
};

enum camera_af_status {
	SENSOR_AF_IN_PROGRESS,
	SENSOR_AF_SUCCESS,
	SENSOR_AF_FAILED
};

struct cam_ctrl_rational_t
{
  uint32_t  numerator;
  uint32_t  denominator;
} ;
/* extension end */

struct cam_ctrl_exif_params_t {
	uint32_t shutter_speed; /* in us */
	uint16_t iso_speed_index;
	uint16_t camera_revision;
	uint8_t  flash_fired; /* extension */
};

/* extension begin */
enum camera_scene
{
	SENSOR_SCENE_AUTO,
	SENSOR_SCENE_MACRO,
	SENSOR_SCENE_TWILIGHT,
	SENSOR_SCENE_SPORTS,
	SENSOR_SCENE_BEACH,
	SENSOR_SCENE_SNOW,
	SENSOR_SCENE_LANDSCAPE,
	SENSOR_SCENE_PORTRAIT,
	SENSOR_SCENE_TWILIGHT_PORTRAIT,
	SENSOR_SCENE_DOCUMENT,
};

enum camera_focus_mode {
	SENSOR_FOCUS_MODE_AUTO,
	SENSOR_FOCUS_MODE_MACRO,
	SENSOR_FOCUS_MODE_CONTINUOUS,
	SENSOR_FOCUS_MODE_FIXED
};
struct wb_info_cfg {
	uint16_t red_gain;
	uint16_t green_gain;
	uint16_t blue_gain;
};

/* extension begin */
enum sensor_gpio_ctrl_type {
	SENSOR_GPIO_CTRL_RESET,
	SENSOR_GPIO_CTRL_STANBY,
};

struct sensor_gpio_ctrl {
	enum sensor_gpio_ctrl_type gpio;
	int value;
};

enum sensor_i2c_addr_type {
	SENSOR_I2C_ADDR_0BYTE = 0,
	SENSOR_I2C_ADDR_1BYTE = 1,
	SENSOR_I2C_ADDR_2BYTE = 2,
	SENSOR_I2C_ADDR_4BYTE = 4,
};

struct sensor_i2c_io {
	uint8_t slave_addr;
	uint32_t address;
	enum sensor_i2c_addr_type address_type;
	uint8_t length;
	uint8_t __user *data;
};

enum sensor_csi_data_format {
	SENSOR_CSI_DATA_8BIT,
	SENSOR_CSI_DATA_10BIT,
	SENSOR_CSI_DATA_12BIT,
};

struct sensor_csi_params {
	enum sensor_csi_data_format data_format;
	uint8_t lane_cnt;
	uint8_t lane_assign;
	uint8_t settle_cnt;
	uint8_t dpcm_scheme;
};

struct sensor_rom_in {
	uint16_t address;
	uint16_t length;
	uint8_t __user *data;
};
/* extension end */

struct sensor_3d_exp_cfg {
	uint16_t gain;
	uint32_t line;
	uint16_t r_gain;
	uint16_t b_gain;
	uint16_t gr_gain;
	uint16_t gb_gain;
	uint16_t gain_adjust;
};
struct sensor_3d_cali_data_t{
	unsigned char left_p_matrix[3][4][8];
	unsigned char right_p_matrix[3][4][8];
	unsigned char square_len[8];
	unsigned char focal_len[8];
	unsigned char pixel_pitch[8];
	uint16_t left_r;
	uint16_t left_b;
	uint16_t left_gb;
	uint16_t left_af_far;
	uint16_t left_af_mid;
	uint16_t left_af_short;
	uint16_t left_af_5um;
	uint16_t left_af_50up;
	uint16_t left_af_50down;
	uint16_t right_r;
	uint16_t right_b;
	uint16_t right_gb;
	uint16_t right_af_far;
	uint16_t right_af_mid;
	uint16_t right_af_short;
	uint16_t right_af_5um;
	uint16_t right_af_50up;
	uint16_t right_af_50down;
};
struct sensor_init_cfg {
	uint8_t prev_res;
	uint8_t pict_res;
};

struct sensor_calib_data {
	/* Color Related Measurements */
	uint16_t r_over_g;
	uint16_t b_over_g;
	uint16_t gr_over_gb;

	/* Lens Related Measurements */
	uint16_t macro_2_inf;
	uint16_t inf_2_macro;
	uint16_t stroke_amt;
	uint16_t af_pos_1m;
	uint16_t af_pos_inf;
};

enum msm_sensor_resolution_t {
	MSM_SENSOR_RES_FULL,
	MSM_SENSOR_RES_QTR,
	MSM_SENSOR_RES_2,
	MSM_SENSOR_RES_3,
	MSM_SENSOR_RES_4,
	MSM_SENSOR_RES_5,
	MSM_SENSOR_RES_6,
	MSM_SENSOR_RES_7,
	MSM_SENSOR_INVALID_RES,
};

struct msm_sensor_output_info_t {
	uint16_t x_output;
	uint16_t y_output;
	uint16_t line_length_pclk;
	uint16_t frame_length_lines;
	uint32_t vt_pixel_clk;
	uint32_t op_pixel_clk;
	uint16_t binning_factor;
};

struct sensor_output_info_t {
	struct msm_sensor_output_info_t *output_info;
	uint16_t num_info;
};

struct sensor_eeprom_data_t {
	void *eeprom_data;
	uint16_t index;
};

struct mirror_flip {
	int32_t x_mirror;
	int32_t y_flip;
};

struct cord {
	uint32_t x;
	uint32_t y;
};

struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t effect;
		uint8_t quality; /* extension */
		uint8_t brightness;
		uint8_t contrast;
		uint8_t sharpness;
		uint8_t wb_type;
		/* extension begin */
		int8_t ev;
		int8_t exp_mode;
		uint16_t iso_mode;
		/* extension end */
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct wb_info_cfg wb_info;
		/* extension begin */
		struct camera_dimension_t dimension;
		enum set_test_pattern_t set_test_pattern;
		enum camera_af_status af_status;
		struct cam_ctrl_exif_params_t exif;
		enum camera_scene scene;
		struct camera_preview_dimension_t preview_dimension;
		uint8_t flashled;
		enum camera_focus_mode focus_mode;
		struct sensor_gpio_ctrl gpio_ctrl;
		struct sensor_i2c_io i2c_io;
		struct sensor_csi_params csi_ctrl;
		struct sensor_rom_in rom_in;
		/* extension end */
	} cfg;
};

struct msm_actuator_move_params_t {
	int8_t dir;
	int32_t num_steps;
};

struct msm_actuator_set_info_t {
	uint32_t total_steps;
	uint16_t gross_steps;
	uint16_t fine_steps;
};

struct msm_actuator_get_info_t {
	uint32_t focal_length_num;
	uint32_t focal_length_den;
	uint32_t f_number_num;
	uint32_t f_number_den;
	uint32_t f_pix_num;
	uint32_t f_pix_den;
	uint32_t total_f_dist_num;
	uint32_t total_f_dist_den;
	uint32_t hor_view_angle_num;
	uint32_t hor_view_angle_den;
	uint32_t ver_view_angle_num;
	uint32_t ver_view_angle_den;
};

struct msm_actuator_cfg_data {
	int cfgtype;
	uint8_t is_af_supported;
	union {
		struct msm_actuator_move_params_t move;
		struct msm_actuator_set_info_t set_info;
		struct msm_actuator_get_info_t get_info;
	} cfg;
};

struct sensor_large_data {
	int cfgtype;
	union {
		struct sensor_3d_cali_data_t sensor_3d_cali_data;
	} data;
};

enum sensor_type_t {
	BAYER,
	YUV,
	JPEG_SOC,
};

enum flash_type {
	LED_FLASH,
	STROBE_FLASH,
};

enum strobe_flash_ctrl_type {
	STROBE_FLASH_CTRL_INIT,
	STROBE_FLASH_CTRL_CHARGE,
	STROBE_FLASH_CTRL_RELEASE
};

struct strobe_flash_ctrl_data {
	enum strobe_flash_ctrl_type type;
	int charge_en;
};

struct msm_camera_info {
	int num_cameras;
	uint8_t has_3d_support[MSM_MAX_CAMERA_SENSORS];
	uint8_t is_internal_cam[MSM_MAX_CAMERA_SENSORS];
	uint32_t s_mount_angle[MSM_MAX_CAMERA_SENSORS];
	const char *video_dev_name[MSM_MAX_CAMERA_SENSORS];
	enum sensor_type_t sensor_type[MSM_MAX_CAMERA_SENSORS];

};

struct msm_cam_config_dev_info {
	int num_config_nodes;
	const char *config_dev_name[MSM_MAX_CAMERA_CONFIGS];
	int config_dev_id[MSM_MAX_CAMERA_CONFIGS];
};

struct msm_mctl_node_info {
	int num_mctl_nodes;
	const char *mctl_node_name[MSM_MAX_CAMERA_SENSORS];
};

struct flash_ctrl_data {
	int flashtype;
	union {
		int led_state;
		struct strobe_flash_ctrl_data strobe_ctrl;
	} ctrl_data;
};

#define GET_NAME			0
#define GET_PREVIEW_LINE_PER_FRAME	1
#define GET_PREVIEW_PIXELS_PER_LINE	2
#define GET_SNAPSHOT_LINE_PER_FRAME	3
#define GET_SNAPSHOT_PIXELS_PER_LINE	4
#define GET_SNAPSHOT_FPS		5
#define GET_SNAPSHOT_MAX_EP_LINE_CNT	6

struct msm_camsensor_info {
	char name[MAX_SENSOR_NAME];
	uint8_t flash_enabled;
	int8_t total_steps;
	uint8_t support_3d;
};

#define V4L2_SINGLE_PLANE	0
#define V4L2_MULTI_PLANE_Y	0
#define V4L2_MULTI_PLANE_CBCR	1
#define V4L2_MULTI_PLANE_CB	1
#define V4L2_MULTI_PLANE_CR	2

struct plane_data {
	int plane_id;
	uint32_t offset;
	unsigned long size;
};

struct img_plane_info {
	uint32_t width;
	uint32_t height;
	uint32_t pixelformat;
	uint8_t buffer_type; /*Single/Multi planar*/
	uint8_t output_port;
	uint32_t ext_mode;
	uint8_t num_planes;
	struct plane_data plane[MAX_PLANES];
	uint32_t sp_y_offset;
	uint8_t vpe_can_use;
};

#endif /* __LINUX_MSM_CAMERA_H */
