// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Force feedback support for various HID compliant devices by ThrustMaster:
 *    ThrustMaster FireStorm Dual Power 2
 * and possibly others whose device ids haven't been added.
 *
 *  Modified to support ThrustMaster devices by Zinx Verituse
 *  on 2003-01-25 from the Logitech force feedback driver,
 *  which is by Johann Deneux.
 *
 *  Copyright (c) 2003 Zinx Verituse <zinx@epicsol.org>
 *  Copyright (c) 2002 Johann Deneux
 */

/*
*/

#include <linux/hid.h>
#include <linux/usb.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/module.h>


#define USB_VENDOR_ID_THRUSTMASTER 0x044f
#define THRUSTMASTER_DEVICE_ID_2_IN_1_DT	0xb320
#define URB_FUNCTION_SYNC_RESET_PIPE_AND_CLEAR_STALL	0x001e

static const signed short ff_rumble[] = {
	FF_RUMBLE,
	-1
};

static const signed short ff_constant[] = {
	FF_CONSTANT,
	-1
};

static const signed short ff_joystick[] = {
	FF_CONSTANT,
	-1
};

#ifdef CONFIG_THRUSTMASTER_FF

/* Usages for thrustmaster devices I know about */
#define THRUSTMASTER_USAGE_FF	(HID_UP_GENDESK | 0xbb)

static u8 setup_0[] = { 0x42, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static u8 setup_1[] = { 0x0a, 0x04, 0x90, 0x03, 0x00, 0x00, 0x00, 0x00 };
static u8 setup_2[] = { 0x0a, 0x04, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00 };
static u8 setup_3[] = { 0x0a, 0x04, 0x12, 0x10, 0x00, 0x00, 0x00, 0x00 };
static u8 setup_4[] = { 0x0a, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00 };
static u8 *setup_arr[] = { setup_0, setup_1, setup_2, setup_3, setup_4 };
static unsigned int setup_arr_sizes[] = { 
	ARRAY_SIZE(setup_0),
	ARRAY_SIZE(setup_1),
	ARRAY_SIZE(setup_2),
	ARRAY_SIZE(setup_3),
	ARRAY_SIZE(setup_4)	
}; 

static u8 hw_rq_in[] = { 0xc1, 0x49, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00 };
static u8 hw_rq_out[] = { 0x41, 0x53, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };

struct tmff_device {
	struct hid_report *report;
	struct hid_field *ff_field;
};

struct api_context {
	struct completion	done;
	int			status;
};

/* Changes values from 0 to 0xffff into values from minimum to maximum */
static inline int tmff_scale_u16(unsigned int in, int minimum, int maximum)
{
	int ret;

	ret = (in * (maximum - minimum) / 0xffff) + minimum;
	if (ret < minimum)
		return minimum;
	if (ret > maximum)
		return maximum;
	return ret;
}

/* Changes values from -0x80 to 0x7f into values from minimum to maximum */
static inline int tmff_scale_s8(int in, int minimum, int maximum)
{
	int ret;

	ret = (((in + 0x80) * (maximum - minimum)) / 0xff) + minimum;
	if (ret < minimum)
		return minimum;
	if (ret > maximum)
		return maximum;
	return ret;
}

static int tmff_play(struct input_dev *dev, void *data,
		struct ff_effect *effect)
{
	struct hid_device *hid = input_get_drvdata(dev);
	struct tmff_device *tmff = data;
	struct hid_field *ff_field = tmff->ff_field;
	int x, y;
	int left, right;	/* Rumbling */
	int motor_swap;

	switch (effect->type) {
		case FF_CONSTANT:
			x = tmff_scale_s8(effect->u.ramp.start_level,
					ff_field->logical_minimum,
					ff_field->logical_maximum);
			y = tmff_scale_s8(effect->u.ramp.end_level,
					ff_field->logical_minimum,
					ff_field->logical_maximum);

			dbg_hid("(x, y)=(%04x, %04x)\n", x, y);
			ff_field->value[0] = x;
			ff_field->value[1] = y;
			hid_hw_request(hid, tmff->report, HID_REQ_SET_REPORT);
			break;

		case FF_RUMBLE:
			left = tmff_scale_u16(effect->u.rumble.weak_magnitude,
					ff_field->logical_minimum,
					ff_field->logical_maximum);
			right = tmff_scale_u16(effect->u.rumble.strong_magnitude,
					ff_field->logical_minimum,
					ff_field->logical_maximum);

			/* 2-in-1 strong motor is left */
			if (hid->product == THRUSTMASTER_DEVICE_ID_2_IN_1_DT) {
				motor_swap = left;
				left = right;
				right = motor_swap;
			}

			dbg_hid("(left,right)=(%08x, %08x)\n", left, right);
			ff_field->value[0] = left;
			ff_field->value[1] = right;
			hid_hw_request(hid, tmff->report, HID_REQ_SET_REPORT);
			break;
	}
	return 0;
}

static void tmff_ctrl(struct urb *urb){
	if(urb->status){
		hid_warn(urb->dev, "urb status %d received\n", urb->status);
	}
}

static int usb_start_wait_urb(struct urb *urb, int timeout, int *actual_length)
{
	struct api_context ctx;
	unsigned long expire;
	int retval;

	init_completion(&ctx.done);
	urb->context = &ctx;
	urb->actual_length = 0;
	retval = usb_submit_urb(urb, GFP_NOIO);
	if (unlikely(retval))
		goto out;

	expire = timeout ? msecs_to_jiffies(timeout) : MAX_SCHEDULE_TIMEOUT;
	if (!wait_for_completion_timeout(&ctx.done, expire)) {
		usb_kill_urb(urb);
		retval = (ctx.status == -ENOENT ? -ETIMEDOUT : ctx.status);

		dev_dbg(&urb->dev->dev,
				"%s timed out on ep%d%s len=%u/%u\n",
				current->comm,
				usb_endpoint_num(&urb->ep->desc),
				usb_urb_dir_in(urb) ? "in" : "out",
				urb->actual_length,
				urb->transfer_buffer_length);
	} else
		retval = ctx.status;
out:
	if (actual_length)
		*actual_length = urb->actual_length;

	usb_free_urb(urb);
	return retval;
}

static int tmff_clear_init(struct hid_device *hid){
	/* die */

	int trans, b_ep, i, err;
	u8 *send_buf, *rq_buf;
	struct device *dev = &hid->dev;
	struct usb_interface *usbif = to_usb_interface(dev->parent);
	struct usb_device *usbdev = interface_to_usbdev(usbif);
	struct usb_host_endpoint *ep;
	struct urb *urb;

	ep = &usbif->cur_altsetting->endpoint[1];
	b_ep = ep->desc.bEndpointAddress;

	send_buf = kmalloc(256, GFP_KERNEL); /* overkill but whatever */
	rq_buf = kmalloc(256, GFP_KERNEL);

	for(i = 0; i < ARRAY_SIZE(setup_arr); ++i){
		memcpy(send_buf, setup_arr[i], setup_arr_sizes[i]);	

		err = usb_interrupt_msg(usbdev,
				usb_sndintpipe(usbdev, b_ep),
				send_buf,
				setup_arr_sizes[i],
				&trans,
				USB_CTRL_SET_TIMEOUT);

		if(err){
			hid_err(hid, "Setup data at index %i couldn't be sent, ERRNO: %i\n", i, err);
			goto error;
		}
	}


	urb = usb_alloc_urb(0, GFP_KERNEL);


	memcpy(send_buf, hw_rq_in, 8);
	memcpy(rq_buf, hw_rq_out, 8);

	usb_fill_control_urb(urb,
			usbdev,
			usb_sndctrlpipe(usbdev, 0),
			rq_buf,
			send_buf,
			0,
			tmff_ctrl,
			hid		
			);

	err = usb_start_wait_urb(urb, USB_CTRL_SET_TIMEOUT, &trans);
	if(err != 0){
		hid_err(hid, "Failed sending ctrl out with ERRNO: %i", err);
		goto error;
	}
	hid_info(hid, "URB submission didn't fail?");	

error:
	kfree(rq_buf);
	kfree(send_buf);
	return err;
}

static int tmff_delete(struct kref *kref){
	/* reserved for future use */
	/* currently the errors we get aren't pretty */	
	
	return 0;
};

static int tmff_init(struct hid_device *hid, const signed short *ff_bits)
{

	struct tmff_device *tmff;
	struct hid_report *report;
	struct list_head *report_list;
	struct hid_input *hidinput = list_entry(hid->inputs.next,
			struct hid_input, list);
	struct input_dev *input_dev = hidinput->input;
	int error;
	int i;
			
	if(hid->product == 0xb65d)
		return tmff_clear_init(hid);

	tmff = kzalloc(sizeof(struct tmff_device), GFP_KERNEL);
	if (!tmff)
		return -ENOMEM;


	/* Find the report to use */
	report_list = &hid->report_enum[HID_OUTPUT_REPORT].report_list;
	list_for_each_entry(report, report_list, list) {
		int fieldnum;

		for (fieldnum = 0; fieldnum < report->maxfield; ++fieldnum) {
			struct hid_field *field = report->field[fieldnum];

			if (field->maxusage <= 0)
				continue;

			switch (field->usage[0].hid) {
				case 0xff00000a:
				case THRUSTMASTER_USAGE_FF:
					if (field->report_count < 2) {
						hid_warn(hid, "ignoring FF field with report_count < 2\n");
						continue;
					}

					if (field->logical_maximum ==
							field->logical_minimum) {
						hid_warn(hid, "ignoring FF field with logical_maximum == logical_minimum\n");
						continue;
					}

					if (tmff->report && tmff->report != report) {
						hid_warn(hid, "ignoring FF field in other report\n");
						continue;
					}

					if (tmff->ff_field && tmff->ff_field != field) {
						hid_warn(hid, "ignoring duplicate FF field\n");
						continue;
					}

					tmff->report = report;
					tmff->ff_field = field;

					for (i = 0; ff_bits[i] >= 0; i++)
						set_bit(ff_bits[i], input_dev->ffbit);

					break;

				default:
					hid_warn(hid, "ignoring unknown output usage %08x\n",
							field->usage[0].hid);
					continue;
			}
		}
	}

	if (!tmff->report) {
		hid_err(hid, "can't find FF field in output reports\n");
		error = -ENODEV;
		goto fail;
	}

	error = input_ff_create_memless(input_dev, tmff, tmff_play);
	if (error)
		goto fail;

	hid_info(hid, "force feedback for ThrustMaster devices by Zinx Verituse <zinx@epicsol.org>\n");
	return 0;

fail:
	kfree(tmff);
	return error;
}
#else
static inline int tmff_init(struct hid_device *hid, const signed short *ff_bits)
{
	return 0;
}
#endif

static int tm_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	printk("We have been retarded");

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_FF);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err;
	}

	tmff_init(hdev, (void *)id->driver_data);

	return 0;
err:
	return ret;
}

static const struct hid_device_id tm_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb300),
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb304),   /* FireStorm Dual Power 2 (and 3) */
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, THRUSTMASTER_DEVICE_ID_2_IN_1_DT),   /* Dual Trigger 2-in-1 */
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb323),   /* Dual Trigger 3-in-1 (PC Mode) */
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb324),   /* Dual Trigger 3-in-1 (PS3 Mode) */
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb605),   /* NASCAR PRO FF2 Wheel */
		.driver_data = (unsigned long)ff_joystick },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb651),	/* FGT Rumble Force Wheel */
		.driver_data = (unsigned long)ff_rumble },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb653),	/* RGT Force Feedback CLUTCH Raging Wheel */
		.driver_data = (unsigned long)ff_joystick },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb654),	/* FGT Force Feedback Wheel */
		.driver_data = (unsigned long)ff_joystick },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb65a),	/* F430 Force Feedback Wheel */
		.driver_data = (unsigned long)ff_joystick },
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb65d), 
		.driver_data = (unsigned long)ff_constant }, /* die */
	{ HID_USB_DEVICE(USB_VENDOR_ID_THRUSTMASTER, 0xb66e),
		.driver_data = (unsigned long)ff_constant },
	{ }
};
MODULE_DEVICE_TABLE(hid, tm_devices);

static struct hid_driver tm_driver = {
	.name = "thrustmaster",
	.id_table = tm_devices,
	.probe = tm_probe,
};
module_hid_driver(tm_driver);

MODULE_LICENSE("GPL");
