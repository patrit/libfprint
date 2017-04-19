/*
 * Elan fingerprint driver for libfprint
 * Copyright (C) 2017 Patrick Ritter <patrit@github.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "elan"


#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <glib.h>
#include <libusb.h>

#include <fp_internal.h>

#include "driver_ids.h"

#define BULK_TIMEOUT	1000
#define EP1_OUT			(1 | LIBUSB_ENDPOINT_OUT)
#define EP2_IN			(2 | LIBUSB_ENDPOINT_IN)
#define EP3_IN			(3 | LIBUSB_ENDPOINT_IN)

#define ENLARGE_FACTOR	 3
#define IMG_WIDTH		96
#define IMG_HEIGHT		96
#define IMG_SIZE		(IMG_WIDTH * IMG_HEIGHT)
#define BUFFER_TRANSMIT 0x2400

struct elan_dev {
	unsigned char buffer[BUFFER_TRANSMIT];
	struct fp_img *capture_img;
	gboolean loop_running;
	gboolean deactivating;
};

enum loop_states {
	INIT_001,
	INIT_002,
	INIT_CMD_SCAN,
	INIT_CAPTURE_1,
	INIT_CAPTURE_2,
	INIT_003,
	INIT_004,
	INIT_005,
	INIT_006,
	INIT_007,
	INIT_008,
	INIT_009,
	INIT_010,
	INIT_011,
	INIT_012,
	INIT_013,
	INIT_014,
	INIT_015,
	LOOP_CMD_SCAN,
	LOOP_CAPTURE_1,
	LOOP_CAPTURE_2,
	LOOP_CAPTURE_DONE,
	LOOP_NUM_STATES,
};

static gboolean finger_is_present(unsigned char *data);

/***** bulk transfer *****/

static void elan_assemble_image(unsigned char *input, unsigned char *output)
{
	size_t row, column;
	size_t idx = 0;

	for (row = 0; row < IMG_HEIGHT / 2; row ++) {
		for (column = 0; column < IMG_WIDTH; column++) {
			uint16_t lower = input[idx*2];
			uint16_t upper = input[idx*2 + 1];
			uint16_t val = (upper * 256 + lower) / 128;
			val = val > 40 ? val - 40 : 0; // min val of 40
			val = val > 64 ? 255 : val * 4;
			output[idx] = val;
			idx++;
		}
	}
}

static void elan_bulk_transfer_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fpi_ssm_mark_aborted(ssm, -EIO);
		libusb_free_transfer(transfer);
		return;
	}

	libusb_free_transfer(transfer);
	fpi_ssm_next_state(ssm);
}


static void elan_bulk_transfer(struct fpi_ssm *ssm, unsigned char endpoint, unsigned char *buffer, int length)
{
	struct fp_img_dev *dev = ssm->priv;
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	int r;

	if (!transfer) {
		fpi_ssm_mark_aborted(ssm, -ENOMEM);
		return;
	}
	
	libusb_fill_bulk_transfer(transfer, dev->udev, endpoint,
		buffer, length,
		elan_bulk_transfer_cb, ssm, BULK_TIMEOUT);
	transfer->flags = 0;
	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		libusb_free_transfer(transfer);
		fpi_ssm_mark_aborted(ssm, r);
	}
}

static void ep3_in_check(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *vdev = dev->priv;
	elan_bulk_transfer(ssm, EP3_IN, vdev->buffer, 0x40);
}

static void ep1_out2(struct fpi_ssm *ssm, unsigned char b1, unsigned char b2)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *vdev = dev->priv;
	vdev->buffer[0] = b1;
	vdev->buffer[1] = b2;
	elan_bulk_transfer(ssm, EP1_OUT, vdev->buffer, 2);
}

static void ep1_out3(struct fpi_ssm *ssm, unsigned char b1, unsigned char b2, unsigned char b3)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *vdev = dev->priv;
	vdev->buffer[0] = b1;
	vdev->buffer[1] = b2;
	vdev->buffer[2] = b3;
	elan_bulk_transfer(ssm, EP1_OUT, vdev->buffer, 3);
}

/***** FINGER DETECTION *****/

/* We take 32x32 pixels at the center of the image, count the number
 * of pixel above a threshold. Expect at least 20% to reach this level */
#define DETBOX_ROW_START 32
#define DETBOX_COL_START 32
#define DETBOX_ROWS 32
#define DETBOX_COLS 32
#define DETBOX_ROW_END (DETBOX_ROW_START + DETBOX_ROWS)
#define DETBOX_COL_END (DETBOX_COL_START + DETBOX_COLS)
#define FINGER_PRESENCE_THRESHOLD 140
#define FINGER_PIXEL_THRESHOLD 200

static gboolean finger_is_present(unsigned char *data)
{
	int row;
	uint8_t num_above_threshold = 0;
	uint16_t num = 0;

	for (row = DETBOX_ROW_START; row < DETBOX_ROW_END; row++) {
		unsigned char *rowdata = data + (row * IMG_WIDTH);
		int col;

		for (col = DETBOX_COL_START; col < DETBOX_COL_END; col++) {
			num++;
			if (rowdata[col] > FINGER_PRESENCE_THRESHOLD) {
				num_above_threshold++;
				if (num_above_threshold > FINGER_PIXEL_THRESHOLD) {
					return TRUE;
				}
			}
		}
	}
	return FALSE;
}

/***** CAPTURE LOOP *****/


static void loop_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *vdev = dev->priv;

	switch (ssm->cur_state) {

	case INIT_001:
		ep1_out2(ssm, 0x00, 0x0c);
		break;
	case INIT_002:
		ep3_in_check(ssm);
		break;
	case INIT_CMD_SCAN:
		ep1_out2(ssm, 0x00, 0x09);
		break;
	case INIT_CAPTURE_1:
		elan_bulk_transfer(ssm, EP2_IN, vdev->buffer, BUFFER_TRANSMIT);
		break;
	case INIT_CAPTURE_2:
		elan_bulk_transfer(ssm, EP2_IN, vdev->buffer, BUFFER_TRANSMIT);
		break;
	case INIT_003:
		ep1_out2(ssm, 0x40, 0x7d);
		break;
	case INIT_004:
		ep3_in_check(ssm);
		break;
	case INIT_005:
		ep1_out3(ssm, 0x40, 0xbd, 0x12);
		break;
	case INIT_006:
		ep1_out3(ssm, 0x40, 0xa8, 0x78);
		break;
	case INIT_007:
		ep1_out2(ssm, 0x40, 0x68);
		break;
	case INIT_008:
		ep3_in_check(ssm);
		break;
	case INIT_009:
		ep1_out2(ssm, 0x40, 0x67);
		break;
	case INIT_010:
		ep3_in_check(ssm);
		break;
	case INIT_011:
		ep1_out2(ssm, 0x40, 0x47);
		break;
	case INIT_012:
		ep3_in_check(ssm);
		break;
	case INIT_013:
		ep1_out3(ssm, 0x40, 0x87, 0xc0);
		break;
	case INIT_014:
		ep1_out3(ssm, 0x40, 0xa8, 0x97);
		break;
	case INIT_015:
		ep1_out3(ssm, 0x40, 0x8b, 0x72);
		break;
	case LOOP_CMD_SCAN:
		if (vdev->deactivating) {
			fp_dbg("deactivating, marking completed");
			fpi_ssm_mark_completed(ssm);
		} else {
			ep1_out2(ssm, 0x00, 0x09);
		}
		break;
	case LOOP_CAPTURE_1: ;
		struct fp_img *img = fpi_img_new(IMG_WIDTH * IMG_HEIGHT);
		img->width = IMG_WIDTH;
		img->height = IMG_HEIGHT;
		img->flags = FP_IMG_COLORS_INVERTED;
		vdev->capture_img = img;
		elan_bulk_transfer(ssm, EP2_IN, vdev->buffer, BUFFER_TRANSMIT);
		break;
	case LOOP_CAPTURE_2:
		elan_assemble_image(vdev->buffer, vdev->capture_img->data);
		elan_bulk_transfer(ssm, EP2_IN, vdev->buffer, BUFFER_TRANSMIT);
		break;
	case LOOP_CAPTURE_DONE:
		elan_assemble_image(vdev->buffer, vdev->capture_img->data + (BUFFER_TRANSMIT / 2));
		fpi_imgdev_report_finger_status(dev, finger_is_present(vdev->capture_img->data));
		fpi_imgdev_image_captured(dev,
				fpi_im_resize(vdev->capture_img, ENLARGE_FACTOR, ENLARGE_FACTOR));
		fp_img_free(vdev->capture_img);
		vdev->capture_img = NULL;
		fpi_ssm_jump_to_state(ssm, LOOP_CMD_SCAN);
		break;
	}
}

static void loopsm_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct elan_dev *vdev = dev->priv;
	int r = ssm->error;

	fpi_ssm_free(ssm);
	fp_img_free(vdev->capture_img);
	vdev->capture_img = NULL;
	vdev->loop_running = FALSE;

	if (r) {
		fpi_imgdev_session_error(dev, r);
    }
	if (vdev->deactivating) {
		fpi_imgdev_deactivate_complete(dev);
    }
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct elan_dev *vdev = dev->priv;
	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, loop_run_state,
		LOOP_NUM_STATES);
	ssm->priv = dev;
	vdev->deactivating = FALSE;
	fpi_ssm_start(ssm, loopsm_complete);
	vdev->loop_running = TRUE;
	fpi_imgdev_activate_complete(dev, 0);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	struct elan_dev *vdev = dev->priv;
	if (vdev->loop_running)
		vdev->deactivating = TRUE;
	else
		fpi_imgdev_deactivate_complete(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	int r;
	dev->priv = g_malloc0(sizeof(struct elan_dev));

	r = libusb_claim_interface(dev->udev, 0);
	if (r < 0)
		fp_err("could not claim interface 0: %s", libusb_error_name(r));

	if (r == 0)
		fpi_imgdev_open_complete(dev, 0);

	return r;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	g_free(dev->priv);
	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x04f3, .product = 0x0c03 },
	{ 0, 0, 0, },
};

struct fp_img_driver elan_driver = {
	.driver = {
		.id = ELAN_ID,
		.name = FP_COMPONENT,
		.full_name = "Elan Fingerprint",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_PRESS,
	},
	.flags = 0,
	.img_height = IMG_HEIGHT * ENLARGE_FACTOR,
	.img_width = IMG_WIDTH * ENLARGE_FACTOR,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};
