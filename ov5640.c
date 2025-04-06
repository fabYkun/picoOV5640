#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "ov5640.h"

const uint16_t _resolution_info[][3] = {
    { 96, 96, ASPECT_RATIO_1X1 }, // 96x96
    { 160, 120, ASPECT_RATIO_4X3 }, // QQVGA
    { 176, 144, ASPECT_RATIO_5X4 }, // QCIF
    { 240, 176, ASPECT_RATIO_4X3 }, // HQVGA
    { 240, 240, ASPECT_RATIO_1X1 }, // 240x240
    { 320, 240, ASPECT_RATIO_4X3 }, // QVGA
    { 400, 296, ASPECT_RATIO_4X3 }, // CIF
    { 480, 320, ASPECT_RATIO_3X2 }, // HVGA
    { 640, 480, ASPECT_RATIO_4X3 }, // VGA
    { 800, 600, ASPECT_RATIO_4X3 }, // SVGA
    { 1024, 768, ASPECT_RATIO_4X3 }, // XGA
    { 1280, 720, ASPECT_RATIO_16X9 }, // HD
    { 1280, 1024, ASPECT_RATIO_5X4 }, // SXGA
    { 1600, 1200, ASPECT_RATIO_4X3 }, // UXGA
    { 2560, 1440, ASPECT_RATIO_16X9 }, // QHD
    { 2560, 1600, ASPECT_RATIO_16X10 }, // WQXGA
    { 1088, 1920, ASPECT_RATIO_9X16 }, // Portrait FHD
    { 2560, 1920, ASPECT_RATIO_4X3 }, // QSXGA
};

const uint16_t _ratio_table[][10] = {
    //  OV5640_RATIO_TABLE
    { 2560, 1920, 0, 0, 2623, 1951, 32, 16, 2844, 1968 }, // 4x3
    { 2560, 1704, 0, 110, 2623, 1843, 32, 16, 2844, 1752 }, // 3x2
    { 2560, 1600, 0, 160, 2623, 1791, 32, 16, 2844, 1648 }, // 16x10
    { 2560, 1536, 0, 192, 2623, 1759, 32, 16, 2844, 1584 }, // 5x3
    { 2560, 1440, 0, 240, 2623, 1711, 32, 16, 2844, 1488 }, // 16x9
    { 2560, 1080, 0, 420, 2623, 1531, 32, 16, 2844, 1128 }, // 21x9
    { 2400, 1920, 80, 0, 2543, 1951, 32, 16, 2684, 1968 }, // 5x4
    { 1920, 1920, 320, 0, 2543, 1951, 32, 16, 2684, 1968 }, // 1x1
    { 1088, 1920, 736, 0, 1887, 1951, 32, 16, 1884, 1968 }, // 9x16
};

const uint8_t _contrast_settings[][2] = {
    { 0x20, 0x00 }, //  0
    { 0x24, 0x10 }, // +1
    { 0x28, 0x18 }, // +2
    { 0x2c, 0x1c }, // +3
    { 0x14, 0x14 }, // -3
    { 0x18, 0x18 }, // -2
    { 0x1c, 0x1c }, // -1
};

const uint8_t _sensor_saturation_levels[][11] = {
    { 0x1D, 0x60, 0x03, 0x0C, 0x78, 0x84, 0x7D, 0x6B, 0x12, 0x01, 0x98 }, // 0
    { 0x1D, 0x60, 0x03, 0x0D, 0x84, 0x91, 0x8A, 0x76, 0x14, 0x01, 0x98 }, // +1
    { 0x1D, 0x60, 0x03, 0x0E, 0x90, 0x9E, 0x96, 0x80, 0x16, 0x01, 0x98 }, // +2
    { 0x1D, 0x60, 0x03, 0x10, 0x9C, 0xAC, 0xA2, 0x8B, 0x17, 0x01, 0x98 }, // +3
    { 0x1D, 0x60, 0x03, 0x11, 0xA8, 0xB9, 0xAF, 0x96, 0x19, 0x01, 0x98 }, // +4
    { 0x1D, 0x60, 0x03, 0x07, 0x48, 0x4F, 0x4B, 0x40, 0x0B, 0x01, 0x98 }, // -4
    { 0x1D, 0x60, 0x03, 0x08, 0x54, 0x5C, 0x58, 0x4B, 0x0D, 0x01, 0x98 }, // -3
    { 0x1D, 0x60, 0x03, 0x0A, 0x60, 0x6A, 0x64, 0x56, 0x0E, 0x01, 0x98 }, // -2
    { 0x1D, 0x60, 0x03, 0x0B, 0x6C, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98 }, // -1
};

const uint8_t _sensor_ev_levels[][6] = {
    { 0x38, 0x30, 0x61, 0x38, 0x30, 0x10 }, //  0
    { 0x40, 0x38, 0x71, 0x40, 0x38, 0x10 }, // +1
    { 0x50, 0x48, 0x90, 0x50, 0x48, 0x20 }, // +2
    { 0x60, 0x58, 0xa0, 0x60, 0x58, 0x20 }, // +3
    { 0x10, 0x08, 0x10, 0x08, 0x20, 0x10 }, // -3
    { 0x20, 0x18, 0x41, 0x20, 0x18, 0x10 }, // -2
    { 0x30, 0x28, 0x61, 0x30, 0x28, 0x10 }, // -1
};

const uint16_t _light_registers[] = { 0x3406, 0x3400, 0x3401, 0x3402, 0x3403, 0x3404, 0x3405 };

const uint16_t _light_modes[][7] = {
    { 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00 }, // auto
    { 0x01, 0x06, 0x1c, 0x04, 0x00, 0x04, 0xf3 }, // sunny
    { 0x01, 0x05, 0x48, 0x04, 0x00, 0x07, 0xcf }, // office / fluorescent
    { 0x01, 0x06, 0x48, 0x04, 0x00, 0x04, 0xd3 }, // cloudy
    { 0x01, 0x04, 0x10, 0x04, 0x00, 0x08, 0x40 }, // home / incandescent
};

const uint16_t _sensor_special_effects[][4] = {
    { 0x06, 0x40, 0x10, 0x08 }, // Normal
    { 0x46, 0x40, 0x28, 0x08 }, // Negative
    { 0x1E, 0x80, 0x80, 0x08 }, // Grayscale
    { 0x1E, 0x80, 0xC0, 0x08 }, // Red Tint
    { 0x1E, 0x60, 0x60, 0x08 }, // Green Tint
    { 0x1E, 0xA0, 0x40, 0x08 }, // Blue Tint
    { 0x1E, 0x40, 0xA0, 0x08 }, // Sepia
};

const uint16_t _sensor_format_rgb565_regs[] = { _FORMAT_CTRL, _FORMAT_CTRL00, _SYSTEM_RESET02, _CLOCK_ENABLE02 };
const uint8_t _sensor_format_rgb565_values[] = { 0x01, 0x61, 0x1C, 0xC3 };

const uint16_t _sensor_format_yuv422_regs[] = { _FORMAT_CTRL, _FORMAT_CTRL00 };
const uint8_t _sensor_format_yuv422_values[] = { 0x00, 0x30 };

const uint16_t _sensor_format_grayscale_regs[] = { _FORMAT_CTRL, _FORMAT_CTRL00 };
const uint8_t _sensor_format_grayscale_values[] = { 0x00, 0x10 };

const uint16_t _sensor_format_jpeg_regs[] = { _FORMAT_CTRL, _FORMAT_CTRL00, _SYSTEM_RESET02, _CLOCK_ENABLE02, 0x471C,
};
const uint8_t _sensor_format_jpeg_values[] = { 0x00, 0x30, 0x00, 0xFF, 0x50 };

const uint16_t _sensor_format_raw_regs[] = { _FORMAT_CTRL, _FORMAT_CTRL00 };
const uint8_t _sensor_format_raw_values[] = { 0x03, 0x00 };

const uint16_t ov5640_init_regs[] = { _SYSTEM_CTROL0, _REG_DLY, _SYSTEM_CTROL0, 0x3103, 0x3017, 0x3018, _DRIVE_CAPABILITY, _CLOCK_POL_CONTROL, 0x4713, _ISP_CONTROL_01, _SYSTEM_RESET00, _SYSTEM_RESET02, 0x3004, _CLOCK_ENABLE02, 0x5000, _ISP_CONTROL_01, 0x5003, 0x370C, 0x3634, 0x3A02, 0x3A03, 0x3A08, 0x3A09, 0x3A0A, 0x3A0B, 0x3A0D, 0x3A0E, 0x3A0F, 0x3A10, 0x3A11, 0x3A13, 0x3A14, 0x3A15, 0x3A18, 0x3A19, 0x3A1B, 0x3A1E, 0x3A1F, 0x3600, 0x3601, 0x3C01, 0x3C04, 0x3C05, 0x3C06, 0x3C07, 0x3C08, 0x3C09, 0x3C0A, 0x3C0B, 0x460C, 0x4001, 0x4004, 0x5180, 0x5181, 0x5182, 0x5183, 0x5184, 0x5185, 0x5186, 0x5187, 0x5188, 0x5189, 0x518A, 0x518B, 0x518C, 0x518D, 0x518E, 0x518F, 0x5190, 0x5191, 0x5192, 0x5193, 0x5194, 0x5195, 0x5196, 0x5197, 0x5198, 0x5199, 0x519A, 0x519B, 0x519C, 0x519D, 0x519E, 0x5381, 0x5382, 0x5383, 0x5384, 0x5385, 0x5386, 0x5387, 0x5388, 0x5389, 0x538A, 0x538B, 0x5300, 0x5301, 0x5302, 0x5303, 0x5304, 0x5305, 0x5306, 0x5307, 0x5308, 0x5309, 0x530A, 0x530B, 0x530C, 0x5480, 0x5481, 0x5482, 0x5483, 0x5484, 0x5485, 0x5486, 0x5487, 0x5488, 0x5489, 0x548A, 0x548B, 0x548C, 0x548D, 0x548E, 0x548F, 0x5490, 0x5580, 0x5583, 0x5584, 0x5586, 0x5587, 0x5588, 0x5589, 0x558A, 0x558B, 0x501D, 0x3008, 0x3C00, _COMPRESSION_CTRL07 };
const uint8_t ov5640_init_values[] = { 0x82, 10, 0x42, 0x13, 0xFF, 0xFF, 0xC3, 0x21, 0x02, 0x83, 0x00, 0x1C, 0xFF, 0xC3, 0xA7, 0xA3, 0x08, 0x02, 0x40, 0x03, 0xD8, 0x01, 0x27, 0x00, 0xF6, 0x04, 0x03, 0x30, 0x28, 0x60, 0x43, 0x03, 0xD8, 0x00, 0xF8, 0x30, 0x26, 0x14, 0x08, 0x33, 0xA4, 0x28, 0x98, 0x00, 0x08, 0x00, 0x1C, 0x9C, 0x40, 0x22, 0x02, 0x02, 0xFF, 0xF2, 0x00, 0x14, 0x25, 0x24, 0x09, 0x09, 0x09, 0x75, 0x54, 0xE0, 0xB2, 0x42, 0x3D, 0x56, 0x46, 0xF8, 0x04, 0x70, 0xF0, 0xF0, 0x03, 0x01, 0x04, 0x12, 0x04, 0x00, 0x06, 0x82, 0x38, 0x1E, 0x5B, 0x08, 0x0A, 0x7E, 0x88, 0x7C, 0x6C, 0x10, 0x01, 0x98, 0x10, 0x10, 0x18, 0x19, 0x10, 0x10, 0x08, 0x16, 0x40, 0x10, 0x10, 0x04, 0x06, 0x01, 0x00, 0x1E, 0x3B, 0x58, 0x66, 0x71, 0x7D, 0x83, 0x8F, 0x98, 0xA6, 0xB8, 0xCA, 0xD7, 0xE3, 0x1D, 0x06, 0x40, 0x10, 0x20, 0x00, 0x00, 0x10, 0x00, 0xF8, 0x40, 0x02, 0x04, 0x10 };

const uint16_t sensor_gamma1_regs[] = { 0x5480, 0x5481, 0x5482, 0x5483, 0x5484, 0x5485, 0x5486, 0x5487, 0x5488, 0x5489, 0x548A, 0x548B, 0x548C, 0x548D, 0x548E, 0x548F, 0x5490 };
const uint8_t sensor_gamma1_values[] = { 0x1, 0x0, 0x1E, 0x3B, 0x58, 0x66, 0x71, 0x7D, 0x83, 0x8F, 0x98, 0xA6, 0xB8, 0xCA, 0xD7, 0xE3, 0x1D };

const uint16_t sensor_awb0_regs[] = { 0x5180, 0x5181, 0x5182, 0x5183, 0x5184, 0x5185, 0x5186, 0x5187, 0x5188, 0x5189, 0x518A, 0x518B, 0x518C, 0x518D, 0x518E, 0x518F, 0x5190, 0x5191, 0x5192, 0x5193, 0x5194, 0x5195, 0x5196, 0x5197, 0x5198, 0x5199, 0x519A, 0x519B, 0x519C, 0x519D, 0x519E };
const uint8_t sensor_awb0_values[] = { 0xFF, 0xF2, 0x00, 0x14, 0x25, 0x24, 0x09, 0x09, 0x09, 0x75, 0x54, 0xE0, 0xB2, 0x42, 0x3D, 0x56, 0x46, 0xF8, 0x04, 0x70, 0xF0, 0xF0, 0x03, 0x01, 0x04, 0x12, 0x04, 0x00, 0x06, 0x82, 0x38 };

const uint16_t _finalize_firmware_load_regs[] = { 0x3022, 0x3023, 0x3024, 0x3025, 0x3026, 0x3027, 0x3028, 0x3029, 0x3000 };
const uint8_t _finalize_firmware_load_values[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00 };

const uint16_t *const regs_list[OV5640_LIST_COUNT] = {
    [OV5640_COLOR_RGB] = _sensor_format_rgb565_regs,
    [OV5640_COLOR_YUV] = _sensor_format_yuv422_regs,
    [OV5640_COLOR_GRAYSCALE] = _sensor_format_grayscale_regs,
    [OV5640_COLOR_JPEG] = _sensor_format_jpeg_regs,
    [OV5640_COLOR_RAW] = _sensor_format_raw_regs,
    
    [OV5640_LIST_INIT] = ov5640_init_regs,
    [OV5640_LIST_GAMMA] = sensor_gamma1_regs,
    [OV5640_LIST_AWB0] = sensor_awb0_regs,
    [OV5640_LIST_FINALIZE_FIRMWARE_LOAD] = _finalize_firmware_load_regs,
};

const uint8_t *const values_list[OV5640_LIST_COUNT] = {
    [OV5640_COLOR_RGB] = _sensor_format_rgb565_values,
    [OV5640_COLOR_YUV] = _sensor_format_yuv422_values,
    [OV5640_COLOR_GRAYSCALE] = _sensor_format_grayscale_values,
    [OV5640_COLOR_JPEG] = _sensor_format_jpeg_values,
    [OV5640_COLOR_RAW] = _sensor_format_raw_values,

    [OV5640_LIST_INIT] = ov5640_init_values,
    [OV5640_LIST_GAMMA] = sensor_gamma1_values,
    [OV5640_LIST_AWB0] = sensor_awb0_values,
    [OV5640_LIST_FINALIZE_FIRMWARE_LOAD] = _finalize_firmware_load_values,
};

const size_t list_sizes[OV5640_LIST_COUNT] = {
    [OV5640_COLOR_RGB] = sizeof(_sensor_format_rgb565_values),
    [OV5640_COLOR_YUV] = sizeof(_sensor_format_yuv422_values),
    [OV5640_COLOR_GRAYSCALE] = sizeof(_sensor_format_grayscale_values),
    [OV5640_COLOR_JPEG] = sizeof(_sensor_format_jpeg_values),
    [OV5640_COLOR_RAW] = sizeof(_sensor_format_raw_values),

    [OV5640_LIST_INIT] = sizeof(ov5640_init_values),
    [OV5640_LIST_GAMMA] = sizeof(sensor_gamma1_values),
    [OV5640_LIST_AWB0] = sizeof(sensor_awb0_values),
    [OV5640_LIST_FINALIZE_FIRMWARE_LOAD] = sizeof(_finalize_firmware_load_values),
};

int inline i2c_ov5640_write(const uint8_t *src, size_t len) {
    return i2c_write_blocking(CAM_I2C, CAM_I2C_ADDR, src, len, false);
}

uint16_t inline _get_chip_id() {
    return _read_register16(_CHIP_ID_HIGH);
}

void _write_register(uint16_t reg, uint8_t value) {
    uint8_t b[] = { reg >> 8, reg & 0xFF, value };
    i2c_ov5640_write(b, sizeof(b));
}

void _write_addr_reg(uint16_t reg, uint16_t x_value, uint16_t y_value) {
    _write_register16(reg, x_value);
    _write_register16(reg + 2, y_value);
}

void _write_register16(uint16_t reg, uint16_t value) {
    _write_register(reg, value >> 8);
    _write_register(reg + 1, value & 0xFF);
}

uint8_t _read_register(uint16_t reg) {
    uint8_t b[] = { reg >> 8, reg & 0xFF };
    i2c_ov5640_write(b, sizeof(b));
    i2c_read_blocking(CAM_I2C, CAM_I2C_ADDR, &b[0], 1, false);
    return b[0];
}

uint16_t _read_register16(uint16_t reg) {
    uint8_t high = _read_register(reg);
    uint8_t low = _read_register(reg + 1);
    return (high << 8) | low;
}

void _write_list(enum OV5640_REGS_LIST_TYPE list) {
    const uint16_t *reg_list = regs_list[list];
    const uint8_t *value_list = values_list[list];
    size_t length = list_sizes[list];
    for (size_t i = 0; i < length; i++) {
        if (reg_list[i] == _REG_DLY) {
            sleep_ms(value_list[i]);
        } else {
            _write_register(reg_list[i], value_list[i]);
        }
    }
}

void _write_reg_bits(uint16_t reg, uint8_t mask, bool enable) {
    uint8_t val = _read_register(reg);
    if (enable) {
        val |= mask;
    } else {
        val &= ~mask;
    }
    _write_register(reg, val);
}

void _set_image_options(bool binning, enum OV5640_COLOR_TYPE colorspace) {
    // Initialize registers
    uint8_t reg20 = 0;
    uint8_t reg21 = 0;
    uint8_t reg4514 = 0;
    uint8_t reg4514_test = 0;

    if (colorspace == OV5640_COLOR_JPEG)
        reg21 |= 0x20;

    if (binning) {
        reg20 |= 1;
        reg21 |= 1;
        reg4514_test |= 4;
    } else
        reg20 |= 0x40;

    if (reg4514_test == 0)
        reg4514 = 0x88;
    else if (reg4514_test == 1)
        reg4514 = 0x00;
    else if (reg4514_test == 2)
        reg4514 = 0xBB;
    else if (reg4514_test == 3)
        reg4514 = 0x00;
    else if (reg4514_test == 4)
        reg4514 = 0xAA;
    else if (reg4514_test == 5)
        reg4514 = 0xBB;
    else if (reg4514_test == 6)
        reg4514 = 0xBB;
    else if (reg4514_test == 7)
        reg4514 = 0xAA;

    _write_register(_TIMING_TC_REG20, reg20);
    _write_register(_TIMING_TC_REG21, reg21);
    _write_register(0x4514, reg4514);

    // Additional configuration based on binning
    if (binning) {
        _write_register(0x4520, 0x0B);
        _write_register(_X_INCREMENT, 0x31);
        _write_register(_Y_INCREMENT, 0x31);
    } else {
        _write_register(0x4520, 0x10);
        _write_register(_X_INCREMENT, 0x11);
        _write_register(_Y_INCREMENT, 0x11);
    }
}

void _set_pll(bool bypass, uint8_t multiplier, uint8_t sys_div, uint8_t pre_div, bool root_2x, uint8_t pclk_root_div, bool pclk_manual, uint8_t pclk_div) {
    if (multiplier > 252 || multiplier < 4 ||
        sys_div > 15 || pre_div > 8 ||
        pclk_div > 31 || pclk_root_div > 3) {
        printf("Invalid argument to internal function\n");
        return;
    }

    _write_register(0x3039, bypass ? 0x80 : 0x00);
    _write_register(0x3034, 0x1A);
    _write_register(0x3035, 1 | ((sys_div & 0xF) << 4));
    _write_register(0x3036, multiplier & 0xFF);
    _write_register(0x3037, (pre_div & 0xF) | (root_2x ? 0x10 : 0x00));
    _write_register(0x3108, ((pclk_root_div & 0x3) << 4) | 0x06);
    _write_register(0x3824, pclk_div & 0x1F);
    _write_register(0x460C, pclk_manual ? 0x22 : 0x22); // ??
    _write_register(0x3103, 0x13);
}

void _set_quality(uint8_t quality) {
    if (quality < 2 || quality > 54) {
        quality = 54;
        printf("Warning: quality should be equal to or included between 2 and 54");
    }
    _write_register(_COMPRESSION_CTRL07, quality & 0x3F); // why the mask
}

size_t _get_buffer_size(enum OV5640_COLOR_TYPE colorspace, enum OV5640_SIZE_TYPE size, uint quality) {
    uint16_t width = _resolution_info[size][0];
    uint16_t height = _resolution_info[size][1];
    if (colorspace == OV5640_COLOR_JPEG) {
        return 2 * width * height / quality; // arbitrary
    } else if (colorspace == OV5640_COLOR_GRAYSCALE) {
        return width * height;
    }
    return width * height * 2;
}

void _set_white_balance(enum OV5640_WHITE_BALANCE_TYPE value) {
    // Validate the input value
    if (value < OV5640_WHITE_BALANCE_AUTO || value > OV5640_WHITE_BALANCE_INCANDESCENT) {
        printf("Invalid exposure value (EV) %d, use one of the OV5640_WHITE_BALANCE_* constants\n", value);
        value = OV5640_WHITE_BALANCE_AUTO;
    }

    _write_register(0x3212, 0x03);
    for (int i = 0; i < 7; i++)
        _write_register(_light_registers[i], _light_modes[value][i]);

    _write_register(0x3212, 0x13);
    _write_register(0x3212, 0xA3);
}

void _set_size_and_colorspace(enum OV5640_SIZE_TYPE size, enum OV5640_COLOR_TYPE colorspace) {
    uint16_t width = _resolution_info[size][0];
    uint16_t height = _resolution_info[size][1];
    enum OV5640_ASPECT aspect = _resolution_info[size][2];
    const uint16_t *rt = _ratio_table[aspect];
    const uint16_t max_width = rt[RATIO_TABLE_mw];
    const uint16_t max_height = rt[RATIO_TABLE_mh];

    bool _binning = (width <= max_width / 2) && (height <= max_height / 2);
    bool _scale = !(
        (width == max_width && height == max_height) ||
        (width == max_width / 2 && height == max_height / 2)
    );

    _write_addr_reg(_X_ADDR_ST_H, rt[RATIO_TABLE_sx], rt[RATIO_TABLE_sy]);
    _write_addr_reg(_X_ADDR_END_H, rt[RATIO_TABLE_ex], rt[RATIO_TABLE_ey]);
    _write_addr_reg(_X_OUTPUT_SIZE_H, width, height);

    if (!_binning) {
        _write_addr_reg(_X_TOTAL_SIZE_H, rt[RATIO_TABLE_tx], rt[RATIO_TABLE_ty]);
        _write_addr_reg(_X_OFFSET_H, rt[RATIO_TABLE_ox], rt[RATIO_TABLE_oy]);
    } else {
        if (width > 920) {
            _write_addr_reg(_X_TOTAL_SIZE_H, rt[RATIO_TABLE_tx] - 200, rt[RATIO_TABLE_ty] / 2);
        } else {
            _write_addr_reg(_X_TOTAL_SIZE_H, 2060, rt[RATIO_TABLE_ty] / 2);
        }
        _write_addr_reg(_X_OFFSET_H, rt[RATIO_TABLE_ox] / 2, rt[RATIO_TABLE_oy] / 2);
    }

    _write_reg_bits(_ISP_CONTROL_01, 0x20, _scale);
    _set_image_options(_binning, colorspace);

    if (colorspace == OV5640_COLOR_JPEG) {
        uint16_t sys_mul = 200;
        if (size < OV5640_SIZE_QVGA)
            sys_mul = 160;
        if (size < OV5640_SIZE_XGA)
            sys_mul = 180;
        _set_pll(false, sys_mul, 4, 2, false, 2, true, 4);
    } else
        _set_pll(false, 32, 1, 1, false, 1, true, 4);

    _write_list((enum OV5640_REGS_LIST_TYPE) colorspace);
}

void power_on() {
    // masterclock - 150 MHz / 7.5 = 20Mhz (MCLK_FREQ)
    clock_gpio_init(MCLK_GPIO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 7.5);

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