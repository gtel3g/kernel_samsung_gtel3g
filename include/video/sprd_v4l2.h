/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SPRD_V4L2_H_
#define _SPRD_V4L2_H_

#define V4L2_TIMING_LEN 16

/*
	capability       parameters                         structure member
	0x1000           capture mode, single or multi      capture.capturemode
	0x1001           skip number for CAP sub-module     capture.reserved[0];
	0x1002           image width/height from sensor     capture.reserved[2], capture.reserved[3];
	0x1003           base id for each frame             capture.reserved[1];

	0x2000           path skip and deci number          recerved[0] channel, [1] deci number
	0x2001           path pause                         recerved[0] channel
	0x2002           path resume                        recerved[0] channel
*/
enum dcam_parm_id {
	CAPTURE_MODE = 0x1000,
	CAPTURE_SKIP_NUM,
	CAPTURE_SENSOR_SIZE,
	CAPTURE_SENSOR_TRIM,
	CAPTURE_FRM_ID_BASE,
	CAPTURE_SET_CROP,
	CAPTURE_SET_FLASH,
	CAPTURE_SET_OUTPUT_SIZE,
	CAPTURE_SET_ZOOM_MODE,
	PATH_FRM_DECI = 0x2000,
	PATH_PAUSE = 0x2001,
	PATH_RESUME = 0x2002,
};

enum
{
	V4L2_TX_DONE  = 0x00,
	V4L2_NO_MEM   = 0x01,
	V4L2_TX_ERR   = 0x02,
	V4L2_CSI2_ERR = 0x03,
	V4L2_SYS_BUSY = 0x04,
	V4L2_TX_CLEAR = 0x05,
	V4L2_TIMEOUT  = 0x10,
	V4L2_TX_STOP  = 0xFF
};

enum if_status {
	IF_OPEN = 0,
	IF_CLOSE
};

enum v4l2_data_endian {
	V4L2_ENDIAN_BIG = 0,
	V4L2_ENDIAN_LITTLE,
	V4L2_ENDIAN_HALFBIG,
	V4L2_ENDIAN_HALFLITTLE,
	V4L2_ENDIAN_MAX
};

#endif //_SPRD_V4L2_H_
