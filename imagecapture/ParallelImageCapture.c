#include <stdio.h>
#include "hardware/pio.h"
#include "ov5640.h"
#include "imagecapture/StateMachine.h"

static rp2pio_statemachine_obj_t self;
const mcu_pin_obj_t data_clock = { .number = PCLK_GPIO };
const mcu_pin_obj_t vertical_sync = { .number = VSYNC_GPIO };
const mcu_pin_obj_t horizontal_reference = { .number = HREF_GPIO };
const mcu_pin_obj_t data = { .number = DATA_GPIO };

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

void common_hal_imagecapture_parallelimagecapture_singleshot_capture(uint8_t *buff, size_t buff_size) {
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