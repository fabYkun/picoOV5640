#include <stdio.h>
#include <malloc.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "ov5640.h"
#include "imagecapture/StateMachine.h"

uint32_t getTotalHeap(void) {
    extern char __StackLimit, __bss_end__;
    
    return &__StackLimit  - &__bss_end__;
 }
 
 uint32_t getFreeHeap(void) {
    struct mallinfo m = mallinfo();
 
    return getTotalHeap() - m.uordblks;
 }

static rp2pio_statemachine_obj_t self;
static mcu_pin_obj_t data_clock = { .number = PCLK_GPIO };
static mcu_pin_obj_t vertical_sync = { .number = VSYNC_GPIO };
static mcu_pin_obj_t horizontal_reference = { .number = HREF_GPIO };
static mcu_pin_obj_t data = { .number = DATA_GPIO };

#pragma region Initialize

void power_on() {
    // Initialize reset/powerdown pins, set their direction to output
	gpio_init(RST_GPIO);
	gpio_set_dir(RST_GPIO, GPIO_OUT);
    gpio_init(PWD_GPIO);
    gpio_set_dir(PWD_GPIO, GPIO_OUT);

	// Procedure copied from adafruit's OV5640 library
	gpio_put(RST_GPIO, 0);
    gpio_put(PWD_GPIO, 1);
	sleep_us(5000);
    gpio_put(PWD_GPIO, 0);
	sleep_us(1000);
	gpio_put(RST_GPIO, 1);
	sleep_us(20000);
}

void init_cam(enum OV5640_SIZE_TYPE size, enum OV5640_COLOR_TYPE colorspace, uint8_t quality) {
    i2c_init(CAM_I2C, CAM_I2C_FREQ);
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);

    _write_list(OV5640_LIST_INIT);

    _set_quality(quality);
    _set_white_balance(OV5640_WHITE_BALANCE_AUTO);
    _set_size_and_colorspace(size, colorspace);

    sleep_ms(300);
}

#pragma endregion
#pragma region Image capture

#define IMAGECAPTURE_CODE(width, pclk, vsync, href) \
    { \
        /* 0 */ pio_encode_wait_gpio(0, vsync), \
        /* 1 */ pio_encode_wait_gpio(1, vsync), \
        /* .wrap_target */  \
        /* 2 */ pio_encode_wait_gpio(1, href), \
        /* 3 */ pio_encode_wait_gpio(1, pclk), \
        /* 4 */ pio_encode_in(pio_pins, width), \
        /* 5 */ pio_encode_wait_gpio(0, pclk), \
        /* .wrap */ \
    }

void common_hal_imagecapture_parallelimagecapture_construct() {
    uint16_t imagecapture_code[] = IMAGECAPTURE_CODE(DATA_COUNT, data_clock.number, vertical_sync.number, horizontal_reference.number);

    common_hal_rp2pio_statemachine_construct(&self,
        imagecapture_code, MP_ARRAY_SIZE(imagecapture_code),
        clock_get_hz(clk_sys), // full speed (4 instructions per loop -> max pclk 30MHz @ 120MHz)
        0, 0, // init
        NULL, 0, // may_exec
        NULL, 0, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // out pins
        &data, DATA_COUNT, // in pins
        PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // in pulls
        NULL, 0, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // set pins
        #if DEBUG_STATE_MACHINE
        &pin_GPIO26, 3, PIO_PINMASK32_FROM_VALUE(7), PIO_PINMASK32_FROM_VALUE(7), // sideset pins
        #else
        NULL, 0, false, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // sideset pins
        #endif
        false, // No sideset enable
        NULL, PULL_NONE, // jump pin
        PIO_PINMASK_OR3(PIO_PINMASK_FROM_PIN(vertical_sync.number), PIO_PINMASK_FROM_PIN(horizontal_reference.number), PIO_PINMASK_FROM_PIN(data_clock.number)),
        // wait gpio pins
        true, // exclusive pin use
        false, 32, false, // out settings
        false, // wait for txstall
        true, 32, true,  // in settings
        false, // Not user-interruptible.
        2, 5, // wrap settings
        PIO_ANY_OFFSET,
        PIO_FIFO_TYPE_DEFAULT,
        PIO_MOV_STATUS_DEFAULT, PIO_MOV_N_DEFAULT);
}

void common_hal_imagecapture_parallelimagecapture_singleshot_capture() {
    size_t buff_size = 2 * 96 * 96 / 10; // _get_buffer_size()
    uint8_t buff[buff_size];

    PIO pio = self.pio;
    uint sm = self.state_machine;
    uint8_t offset = rp2pio_statemachine_program_offset(&self);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    transfer_in(&self, buff, buff_size, 4, false);

    pio_sm_set_enabled(pio, sm, false);

    printf("Finished capture_image()\n");
    for (size_t i = 0; i < 10; i++) {
        printf("%d ", buff[i]);
    }
    printf("\n");
    for (size_t i = 0; i < buff_size - 1; i++) {
        if (buff[i] == 0xFF && buff[i+1] == 0xD9) {
            // Return the length up to and including the marker
            printf("End of jpg at index %d\n", i + 2);
        }
    }
}

#pragma endregion

int main()
{
    uint32_t ram = getFreeHeap();
    // uart logs
    stdio_init_all();

    printf("total memory: %d\n", getTotalHeap());
    printf("memory: %d\n", ram);

    // masterclock
    clock_gpio_init(MCLK_GPIO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 7.5);

    sleep_ms(300); // make sure camera is "on"
    // rst and pwd pins
    power_on();

    init_cam(OV5640_SIZE_96X96, OV5640_COLOR_JPEG, 10);

    sleep_ms(300);
    printf("Camera initialized\n");

    common_hal_imagecapture_parallelimagecapture_construct();
    common_hal_imagecapture_parallelimagecapture_singleshot_capture();
    
    printf("\n");

    printf("memory: %d\n", getFreeHeap());

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
