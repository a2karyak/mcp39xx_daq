#ifndef USB_DEVICE_H_
#define USB_DEVICE_H_

enum
{
	INTERFACE_COMM,
	INTERFACE_DATA,
	_INTERFACE_NUM,
};

enum
{
	ENDPOINT_CTRL_INTERRUPT = 1,
	ENDPOINT_DATA_RX,
	ENDPOINT_DATA_TX,
	_ENDPOINT_NUM,
};

enum
{
	USB_PM_RX = USB_PM_APP,
	USB_PM_RX_NUM_PKT = 1,
	USB_PM_TX = USB_PM_RX + USB_PM_RX_NUM_PKT * 64,
	USB_PM_TX_NUM_PKT = 2,
	USB_PM_LEN = 64 * USB_PM_TX_NUM_PKT,
};

const uint8_t *usb_get_descriptor(struct usb_device *dev, unsigned type, unsigned index, unsigned lang_id,
		unsigned *len);

struct usb_setup_packet;
uint8_t *usb_control_request(
		struct usb_device *dev,
		const struct usb_setup_packet *setup_packet,
		uint16_t *len,
		usb_control_callback *cb);

#endif
