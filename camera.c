#include <stdio.h>
#include "pico/stdlib.h"
#include "ov5640.h"
#include "imagecapture/ParallelImageCapture.h"

int main()
{
    // uart logs
    stdio_init_all();

    power_on();

    init_cam(OV5640_SIZE_96X96, OV5640_COLOR_JPEG, 10);

    sleep_ms(300);
    printf("Camera initialized\n");

    common_hal_imagecapture_parallelimagecapture_construct();

    size_t buff_size = 2 * 96 * 96 / 10; // _get_buffer_size()
    uint8_t buff[buff_size];
    printf("%d\n", _resolution_info[0][0]);
    common_hal_imagecapture_parallelimagecapture_singleshot_capture(&buff[0], buff_size);
    
    printf("\n");

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
