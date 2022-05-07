#ifndef __USB_TRANSPORTER_H__
#define __USB_TRANSPORTER_H__

#include "struct_typedef.h"
#include "main.h"
#include "SkiderPackage.h"



extern void usbcdc_message_transmit(void);
extern void usbcdc_write_buffer_imu_gyro_frame(uint8_t *imu_gyro_raw_buff);
extern void usbcdc_write_buffer_imu_accl_frame(uint8_t *imu_accl_raw_buff);
extern void usbcdc_write_buffer_remote_sbus_frame(uint8_t *remote_sbus_buff);



#endif
