#include <usb.h>
#include <usb_desc.h>
#include <usb_def.h>
#include <usb_cdc.h>
#include "usb_device.h"

#include <assert.h>

enum
{
	STRING_ID_MANUFACTURER,
	STRING_ID_PRODUCT,
	STRING_ID_SERIAL,
	NUM_STRINGS,
};

enum
{
	CONFIGURATION = 1,
};

static const char *STRINGS[NUM_STRINGS] =
{
	"CDC-ACM Device",
	"CDC-ACM Device",
	"0",
};

static struct usb_cdc_config cdc_cfg =
{
	.comm_interface_id = INTERFACE_COMM,
	.comm_interface_string = 0,
	.comm_int_ep_addr = ENDPOINT_CTRL_INTERRUPT,
	.comm_int_ep_interval = 255,
	.data_interface_id = INTERFACE_DATA,
	.data_interface_string = 0,
	.data_rx_ep_addr = ENDPOINT_DATA_RX,
	.data_tx_ep_addr = ENDPOINT_DATA_TX,
};

enum
{
	USB_EP0_BUF_SZ = 128, // large enough for any transaction or descriptor
};

static uint8_t ep0_buffer[USB_EP0_BUF_SZ];

void usb_connected(struct usb_device *dev);

static uint8_t *get_device_desc(uint8_t *p, uint8_t *end)
{
	p = build_desc_hdr(p, end, USB_DESC_DEVICE, sizeof(struct usb_descriptor_device));
	struct usb_descriptor_device *p_dev = (struct usb_descriptor_device *) p;
	p_dev->bcdUSB = 0x200;
	p_dev->bDeviceClass = 0; // per-interface configuration
	p_dev->bDeviceSubClass = 0;
	p_dev->bDeviceProtocol = 0;
	p_dev->bMaxPacketSize0 = USB_MAX_PACKET_EP0;
	p_dev->idVendor = 0x0483; // TODO
	p_dev->idProduct = 0x5740; // TODO
	p_dev->bcdDevice = 0x200;
	p_dev->iManufacturer = STRING_ID_MANUFACTURER + 1;
	p_dev->iProduct = STRING_ID_PRODUCT + 1;
	p_dev->iSerialNumber = STRING_ID_SERIAL + 1;
	p_dev->bNumConfigurations = 1;
	p += sizeof(struct usb_descriptor_device);
	return p;
}

static uint8_t *get_configuration_desc(uint8_t *p, uint8_t *end)
{
	p = build_desc_hdr(p, end, USB_DESC_CONFIGURATION, sizeof(struct usb_descriptor_configuration));
	struct usb_descriptor_configuration *p_cfg = (struct usb_descriptor_configuration *) p;
	p_cfg->bNumInterfaces = _INTERFACE_NUM;
	p_cfg->bConfigurationValue = CONFIGURATION;
	p_cfg->iConfiguration = 0;
	p_cfg->bmAttributes = 0x80; //  bit 7 is set for historical reasons
	p_cfg->bMaxPower = 100 / 2; // 2 mA units
	p += sizeof(struct usb_descriptor_configuration);
	// only one alternate setting in each interface
	p = usb_cdc_build_comm_interface_desc(p, end, &cdc_cfg);
	p = usb_cdc_build_data_interface_desc(p, end, &cdc_cfg);
	p_cfg->wTotalLength = p - ep0_buffer;
	return p;
}

#ifndef PRINT_ONLY
uint8_t *usb_control_request_out(
		struct usb_device *dev,
		const struct usb_setup_packet *setup_packet,
		usb_control_callback *cb)
{
	if ((setup_packet->bmRequestType & USB_REQ_TYPE_TYPE_MASK) != USB_REQ_TYPE_TYPE_STANDARD)
		return 0;

	assert(setup_packet->wLength <= USB_EP0_BUF_SZ);
	if (setup_packet->wLength > USB_EP0_BUF_SZ)
		return 0;

	*cb = 0;
	unsigned recipient = setup_packet->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK;
	switch (recipient)
	{
	case USB_REQ_TYPE_RECIPIENT_DEVICE:
		switch (setup_packet->bRequest)
		{
		case USB_REQ_SET_ADDRESS:
			if (setup_packet->bmRequestType != 0 || setup_packet->wValue >= 128)
				return 0;
			usb_drv_set_address(dev->driver, setup_packet->wValue);
			break;
		case USB_REQ_SET_CONFIGURATION:
			if (setup_packet->wValue != CONFIGURATION)
				return 0;
			if (setup_packet->wValue != dev->current_config)
			{
				dev->current_config = setup_packet->wValue;
				usb_cdc_set_configuration(dev, &cdc_cfg);
				usb_connected(dev);
			}
			break;
		default:
			return 0;
		}
		break;
	case USB_REQ_TYPE_RECIPIENT_INTERFACE:
	{
		unsigned interface = setup_packet->wIndex;
		switch (interface)
		{
		case INTERFACE_COMM:
			if (!usb_cdc_comm_interface_request_out(dev, setup_packet, cb))
				return 0;
			break;
		case INTERFACE_DATA:
			if (!usb_cdc_data_interface_request_out(dev, setup_packet, cb))
				return 0;
			break;
		default:
			return 0;
		}
		break;
	}
	case USB_REQ_TYPE_RECIPIENT_ENDPOINT:
		if (!usb_standard_endpoint_request_out(dev, setup_packet, cb))
			return 0;
		break;
	default:
		return 0;
	}
	return ep0_buffer;
}

const uint8_t *usb_control_request_in(
		struct usb_device *dev,
		const struct usb_setup_packet *setup_packet,
		uint16_t *len)
{
	if ((setup_packet->bmRequestType & USB_REQ_TYPE_TYPE_MASK) != USB_REQ_TYPE_TYPE_STANDARD)
		return 0;

	uint8_t *p = ep0_buffer;
	uint8_t *end = ep0_buffer + USB_EP0_BUF_SZ;
	const uint8_t *ret;

	unsigned recipient = setup_packet->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK;
	switch (recipient)
	{
	case USB_REQ_TYPE_RECIPIENT_DEVICE:
		switch (setup_packet->bRequest)
		{
		case USB_REQ_GET_STATUS:
			*p++ = 0;
			*p++ = 0;
			ret = p;
			break;
		case USB_REQ_GET_DESCRIPTOR:
		{
			unsigned desc_type = setup_packet->wValue >> 8;
			unsigned desc_index = setup_packet->wValue & 0xff;
			switch (desc_type)
			{
			case USB_DESC_DEVICE:
				ret = get_device_desc(p, end);
				break;
			case USB_DESC_CONFIGURATION:
				ret = get_configuration_desc(p, end);
				break;
			case USB_DESC_STRING:
				ret = build_string_desc(p, end, NUM_STRINGS, STRINGS, desc_index, setup_packet->wIndex);
				break;
			default:
				return 0;
			}
			break;
		}
		case USB_REQ_GET_CONFIGURATION:
			assert(p + 1 >= end);
			*p++ = dev->current_config;
			ret = p;
			break;
		default:
			return 0;
		}
		break;
	case USB_REQ_TYPE_RECIPIENT_INTERFACE:
	{
		unsigned interface = setup_packet->wIndex;
		switch (interface)
		{
		case INTERFACE_COMM:
			ret = usb_cdc_comm_interface_request_in(dev, setup_packet, p, end);
			break;
		case INTERFACE_DATA:
			ret = usb_cdc_data_interface_request_in(dev, setup_packet, p, end);
			break;
		default:
			return 0;
		}
		break;
	}
	case USB_REQ_TYPE_RECIPIENT_ENDPOINT:
		ret = usb_standard_endpoint_request_in(dev, setup_packet, p, end);
		break;
	default:
		return 0;
	}
	if (ret != 0)
		*len = ret - ep0_buffer;
	return ep0_buffer;
}
#endif
