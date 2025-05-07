#include <stdio.h>
#include "pico/stdlib.h"
#include "ov5640.h"

#include "imagecapture/StateMachine.h"
#include "imagecapture/ParallelImageCapture.h"

static rp2pio_statemachine_obj_t cam1_sm;
mcu_pin_obj_t cam1_data = { .number = CAM1_DATA_GPIO };
const camera_settings_t cam1_settings = {
    .mclk_gpio = CAM1_MCLK_GPIO,
    .sda_gpio = CAM1_SDA_GPIO,
    .scl_gpio = CAM1_SCL_GPIO,
    .pwd_gpio = CAM1_PWD_GPIO,
    .rst_gpio = CAM1_RST_GPIO,
    .i2c = CAM1_I2C
};
const camera_sm_params_t cam1_sm_params = {
    .pclk_gpio = CAM1_PCLK_GPIO,
    .vsync_gpio = CAM1_VSYNC_GPIO,
    .href_gpio = CAM1_HREF_GPIO,
};

// change the function to your needs
void start_camera(rp2pio_statemachine_obj_t *sm, mcu_pin_obj_t *data_pin, const camera_settings_t *settings, const camera_sm_params_t *sm_params, char id) {
    power_on(settings);
    sleep_us(5000);
    init_cam(settings, OV5640_SIZE_SVGA, OV5640_COLOR_JPEG, 2);
    sleep_us(5000);
    common_hal_imagecapture_parallelimagecapture_construct(sm, data_pin, sm_params->pclk_gpio, sm_params->vsync_gpio, sm_params->href_gpio);
    printf("Camera %c ON\n", id);
}

int main()
{
    // uart logs
    stdio_init_all();

    // will initialize camera & state machine settings for captures
    start_camera(&cam1_sm, &cam1_data, &cam1_settings, &cam1_sm_params, '1');

    printf("Camera capture attempt. \n");
    size_t buff_size = 2 * 96 * 96 / 10; // _get_buffer_size()
    uint8_t buff[buff_size];
    common_hal_imagecapture_parallelimagecapture_singleshot_capture(&cam1_sm, &buff[0], buff_size, true);
}
