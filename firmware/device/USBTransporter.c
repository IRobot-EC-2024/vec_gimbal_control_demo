#include "USBTransporter.h"
#include "string.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

SkiderFeedbackFrame_t SkiderFeedbackFrame;


void usbcdc_message_transmit(void)
{
    SkiderFeedbackFrame._SOF_ = 0x66;
    SkiderFeedbackFrame._EOF_ = 0x88;
    CDC_Transmit_FS((uint8_t *)&SkiderFeedbackFrame, sizeof(SkiderFeedbackFrame_t));
}

void usbcdc_write_buffer_imu_gyro_frame(uint8_t *imu_gyro_raw_buff)
{
    memcpy(&SkiderFeedbackFrame.Gyro, imu_gyro_raw_buff, 6);
}
void usbcdc_write_buffer_imu_accl_frame(uint8_t *imu_accl_raw_buff)
{
    memcpy(&SkiderFeedbackFrame.Accl, imu_accl_raw_buff, 6);
}
void usbcdc_write_buffer_remote_sbus_frame(uint8_t *remote_sbus_buff)
{
    memcpy(&SkiderFeedbackFrame.Sbus, remote_sbus_buff, 18);
}









