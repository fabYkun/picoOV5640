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

size_t __no_inline_not_in_flash_func(init_psram_pimoroni)() {
	printf("Initialize psram\n");
	// RP2350 QMI PSRAM initialization code from CircuitPython

	gpio_set_function(47, GPIO_FUNC_XIP_CS1);
	int psram_size;
	psram_size = 0;

	uint32_t interrupt_state = save_and_disable_interrupts();

	// Try and read the PSRAM ID via direct_csr.
	qmi_hw->direct_csr =
		30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS;
	// Need to poll for the cooldown on the last XIP transfer to expire
	// (via direct-mode BUSY flag) before it is safe to perform the first
	// direct-mode operation
	while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
		;

	// Exit out of QMI in case we've inited already
	qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
	// Transmit as quad.
	qmi_hw->direct_tx =
		QMI_DIRECT_TX_OE_BITS |
		QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB | 0xf5;
	while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
		;
	(void)qmi_hw->direct_rx;
	qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);

	// Read the id
	qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
	uint8_t kgd = 0;
	uint8_t eid = 0;
	for (size_t i = 0; i < 7; i++) {
		if (i == 0) {
			qmi_hw->direct_tx = 0x9f;
		} else {
			qmi_hw->direct_tx = 0xff;
		}
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) {
		}
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
		}
		if (i == 5) {
			kgd = qmi_hw->direct_rx;
		} else if (i == 6) {
			eid = qmi_hw->direct_rx;
		} else {
			(void)qmi_hw->direct_rx;
		}
	}
	// Disable direct csr.
	qmi_hw->direct_csr &=
		~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

	// printf("kgd: %02x\n", kgd);
	/*if (kgd != 0x5D) {
	    common_hal_mcu_enable_interrupts();
	    reset_pin_number(CIRCUITPY_PSRAM_CHIP_SELECT->number);
	    return;
	}
	never_reset_pin_number(CIRCUITPY_PSRAM_CHIP_SELECT->number);*/

	// Enable quad mode.
	qmi_hw->direct_csr =
		30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS;
	// Need to poll for the cooldown on the last XIP transfer to expire
	// (via direct-mode BUSY flag) before it is safe to perform the first
	// direct-mode operation
	while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
	}

	// RESETEN, RESET and quad enable
	for (uint8_t i = 0; i < 3; i++) {
		qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
		if (i == 0) {
			qmi_hw->direct_tx = 0x66;
		} else if (i == 1) {
			qmi_hw->direct_tx = 0x99;
		} else {
			qmi_hw->direct_tx = 0x35;
		}
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
		}
		qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);
		for (size_t j = 0; j < 20; j++) {
			asm("nop");
		}
		(void)qmi_hw->direct_rx;
	}
	// Disable direct csr.
	qmi_hw->direct_csr &=
		~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

	qmi_hw->m[1].timing =
		QMI_M0_TIMING_PAGEBREAK_VALUE_1024
			<< QMI_M0_TIMING_PAGEBREAK_LSB | // Break between pages.
		3 << QMI_M0_TIMING_SELECT_HOLD_LSB | // Delay releasing CS for 3 extra
	                                         // system cycles.
		3 << QMI_M0_TIMING_COOLDOWN_LSB |
		3 << QMI_M0_TIMING_RXDELAY_LSB |
		32 << QMI_M0_TIMING_MAX_SELECT_LSB | // In units of 64 system clock
	                                         // cycles. PSRAM says 8us max. 8 /
	                                         // 0.00752 / 64 = 16.62
		14 << QMI_M0_TIMING_MIN_DESELECT_LSB | // In units of system clock
	                                           // cycles. PSRAM says 50ns.50
	                                           // / 7.52 = 6.64
		3 << QMI_M0_TIMING_CLKDIV_LSB;
	qmi_hw->m[1].rfmt =
		(QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |
	     QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB |
	     QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |
	     QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB |
	     QMI_M0_RFMT_DUMMY_LEN_VALUE_24 << QMI_M0_RFMT_DUMMY_LEN_LSB |
	     QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB |
	     QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB |
	     QMI_M0_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_RFMT_SUFFIX_LEN_LSB);
	qmi_hw->m[1].rcmd =
		0xeb << QMI_M0_RCMD_PREFIX_LSB | 0 << QMI_M0_RCMD_SUFFIX_LSB;
	qmi_hw->m[1].wfmt =
		(QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |
	     QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB |
	     QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |
	     QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB |
	     QMI_M0_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M0_WFMT_DUMMY_LEN_LSB |
	     QMI_M0_WFMT_DATA_WIDTH_VALUE_Q << QMI_M0_WFMT_DATA_WIDTH_LSB |
	     QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB |
	     QMI_M0_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_WFMT_SUFFIX_LEN_LSB);
	qmi_hw->m[1].wcmd =
		0x38 << QMI_M0_WCMD_PREFIX_LSB | 0 << QMI_M0_WCMD_SUFFIX_LSB;

	restore_interrupts(interrupt_state);

	psram_size = 1024 * 1024; // 1 MiB
	uint8_t size_id = eid >> 5;
	if (eid == 0x26 || size_id == 2) {
		psram_size *= 8;
	} else if (size_id == 0) {
		psram_size *= 2;
	} else if (size_id == 1) {
		psram_size *= 4;
	}

    printf("psram size %d\n", psram_size);

	// Mark that we can write to PSRAM.
	xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;

#if DEBUG_PSRAM_MEMCHECK
	// half-assed PSRAM test code
	printf("Writing test pattern to PSRAM (through XIP)\n");
	for (int i = psram_size / 4 - 1; i >= 0; --i) {
		if (i % 8192 == 0) {
			printf(".");
		}

		volatile uint32_t *test_address =
			(uint32_t *)(PSRAM_BASE_CAM1 + i * 4);

		*test_address = (uint32_t)(i);
	}

	printf("\nVerifying test pattern from PSRAM (through XIP)\n");
	for (int i = 0; i < psram_size / 4; ++i) {
		if (i % 8192 == 0) {
			printf(".");
		}

		volatile uint32_t *test_address =
			(uint32_t *)(PSRAM_BASE_CAM1 + i * 4);
		const uint32_t got = *test_address;
		if (got != (uint32_t)(i)) {
			printf("\nPSRAM self test failed (got %d expected %d at %p)! PSRAM "
			       "heap will be disabled.\n",
			       got, (uint32_t)(i), test_address);
			return 0;
		}
	}
	printf("\nPSRAM self test passed!\n");
#endif

	return psram_size;
}

int main()
{
    // uart logs
    stdio_init_all();

    #if ENABLE_PSRAM
        init_psram_pimoroni();
    #endif

    // will initialize camera & state machine settings for captures
    start_camera(&cam1_sm, &cam1_data, &cam1_settings, &cam1_sm_params, '1');

    printf("Camera capture attempt. \n");
    size_t buff_size = 2 * 96 * 96 / 10; // _get_buffer_size()
    uint8_t buff[buff_size];
    common_hal_imagecapture_parallelimagecapture_singleshot_capture(&cam1_sm, &buff[0], buff_size, true);
}
