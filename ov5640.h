// pins
#define MCLK_GPIO 21 // also called XCLK, "master" clock
#define SDA_GPIO 2
#define SCL_GPIO 3
#define PWD_GPIO 14
#define RST_GPIO 15
#define DATA_GPIO 6 // first data pin connection (labelled D2 on OV5640 module)
#define DATA_COUNT 8 // how many data pins there is - they should all be in line / next to DATA_GPIO
#define PCLK_GPIO 26 // pixel clock
#define VSYNC_GPIO 27
#define HREF_GPIO 4

// MasterClock mclk
#define MCLK_FREQ 20000000 // 20 000 000 value used by adafruit's OV5640 library - 25000000 is what's used by the camera chip itself when auto-clockmastering
# define MCLK_DUTY_CYCLE 50 // 32768 value used by adafruit's OV5640 library = 50% (half of 16bit)

// I2C defines
#define CAM_I2C i2c1
#define CAM_I2C_ADDR 0x3C
#define CAM_I2C_FREQ 100000

// OV5640 option enums
enum OV5640_SIZE_TYPE {
    OV5640_SIZE_96X96 = 0, OV5640_SIZE_QQVGA, OV5640_SIZE_QCIF, OV5640_SIZE_HQVGA, OV5640_SIZE_240X240, OV5640_SIZE_QVGA, OV5640_SIZE_CIF, OV5640_SIZE_HVGA, OV5640_SIZE_VGA, OV5640_SIZE_SVGA, OV5640_SIZE_XGA, OV5640_SIZE_HD, OV5640_SIZE_SXGA, OV5640_SIZE_UXGA, OV5640_SIZE_QHDA, OV5640_SIZE_WQXGA, OV5640_SIZE_PFHD, OV5640_SIZE_QSXGA,
    
    OV5640_SIZE_COUNT
};

enum OV5640_ASPECT {
    ASPECT_RATIO_4X3 = 0, ASPECT_RATIO_3X2, ASPECT_RATIO_16X10, ASPECT_RATIO_5X3, ASPECT_RATIO_16X9, ASPECT_RATIO_21X9, ASPECT_RATIO_5X4, ASPECT_RATIO_1X1, ASPECT_RATIO_9X16,

    ASPECT_RATIO_COUNT
};

enum OV5640_WHITE_BALANCE_TYPE {
    OV5640_WHITE_BALANCE_AUTO = 0, OV5640_WHITE_BALANCE_SUNNY, OV5640_WHITE_BALANCE_FLUORESCENT, OV5640_WHITE_BALANCE_CLOUDY, OV5640_WHITE_BALANCE_INCANDESCENT, 

    OV5640_WHITE_BALANCE_COUNT
};

enum OV5640_SPECIAL_EFFECT_TYPE {
    OV5640_SPECIAL_EFFECT_NONE = 0, OV5640_SPECIAL_EFFECT_NEGATIVE, OV5640_SPECIAL_EFFECT_GRAYSCALE, OV5640_SPECIAL_EFFECT_RED_TINT, OV5640_SPECIAL_EFFECT_GREEN_TINT, OV5640_SPECIAL_EFFECT_BLUE_TINT, OV5640_SPECIAL_EFFECT_SEPIA, 

    OV5640_SPECIAL_EFFECT_COUNT
};

enum OV5640_RATIO_TABLE {
    RATIO_TABLE_mw = 0, RATIO_TABLE_mh, RATIO_TABLE_sx, RATIO_TABLE_sy, RATIO_TABLE_ex, RATIO_TABLE_ey, RATIO_TABLE_ox, RATIO_TABLE_oy, RATIO_TABLE_tx, RATIO_TABLE_ty
};

enum OV5640_COLOR_TYPE {
    OV5640_COLOR_RGB = 0, OV5640_COLOR_YUV, OV5640_COLOR_GRAYSCALE, OV5640_COLOR_JPEG,
    OV5640_COLOR_RAW,
    
    OV5640_COLOR_COUNT
};

// default OV5640 REGISTERS/VALUE lists
enum OV5640_REGS_LIST_TYPE {
    /* COLOR REGS */
    OV5640_LIST_INIT = OV5640_COLOR_COUNT,
    OV5640_LIST_GAMMA, OV5640_LIST_AWB0, OV5640_LIST_FINALIZE_FIRMWARE_LOAD,

    OV5640_LIST_COUNT
};

extern const uint16_t _resolution_info[][3];

#define GET_JPG_BUFFER_SIZE(size, quality) ((2 * _resolution_info[size][0] * _resolution_info[size][1]) / quality)
#define GET_GRAYSCALE_BUFFER_SIZE(size) (_resolution_info[size][0] * _resolution_info[size][1])
#define GET_BUFFER_SIZE(size) (2 * _resolution_info[size][0] * _resolution_info[size][1])

// interface functions
void _write_register(uint16_t reg, uint8_t value);
void _write_addr_reg(uint16_t reg, uint16_t x_value, uint16_t y_value);
void _write_register16(uint16_t reg, uint16_t value);
uint8_t _read_register(uint16_t reg);
uint16_t _read_register16(uint16_t reg);
void _write_list(enum OV5640_REGS_LIST_TYPE list);
void _write_reg_bits(uint16_t reg, uint8_t mask, bool enable);
uint16_t _get_chip_id();

void _set_quality(uint8_t quality);
void _set_white_balance(enum OV5640_WHITE_BALANCE_TYPE value);
void _set_size_and_colorspace(enum OV5640_SIZE_TYPE size, enum OV5640_COLOR_TYPE colorspace);
size_t _get_buffer_size(enum OV5640_COLOR_TYPE colorspace, enum OV5640_SIZE_TYPE size, uint quality);

void power_on();
void init_cam(enum OV5640_SIZE_TYPE size, enum OV5640_COLOR_TYPE colorspace, uint8_t quality);

// I2C constants
#define _SYSTEM_RESET00 0x3000
#define _SYSTEM_RESET02 0x3002
#define _CLOCK_ENABLE02 0x3006
#define _SYSTEM_CTROL0 0x3008
#define _CHIP_ID_HIGH 0x300A
#define _DRIVE_CAPABILITY 0x302C

#define _X_ADDR_ST_H 0x3800
#define _X_ADDR_END_H 0x3804
#define _X_OUTPUT_SIZE_H 0x3808
#define _X_TOTAL_SIZE_H 0x380C
#define _X_OFFSET_H 0x3810
#define _X_INCREMENT 0x3814
#define _Y_INCREMENT 0x3815
#define _TIMING_TC_REG20 0x3820
#define _TIMING_TC_REG21 0x3821
#define _FORMAT_CTRL00 0x4300
#define _CLOCK_POL_CONTROL 0x4740
#define _ISP_CONTROL_01 0x5001
#define _FORMAT_CTRL 0x501F
#define _PRE_ISP_TEST_SETTING_1 0x503D
#define _COMPRESSION_CTRL07 0x4407

#define _REG_DLY 0xFFFF
#define _REGLIST_TAIL 0x0000
#define _OV5640_STAT_FIRMWAREBAD 0x7F
#define _OV5640_STAT_STARTUP 0x7E
#define _OV5640_STAT_IDLE 0x70
#define _OV5640_STAT_FOCUSING 0x00
#define _OV5640_STAT_FOCUSED 0x10
#define _OV5640_CMD_TRIGGER_AUTOFOCUS 0x03
#define _OV5640_CMD_AUTO_AUTOFOCUS 0x04
#define _OV5640_CMD_RELEASE_FOCUS 0x08
#define _OV5640_CMD_AF_SET_VCM_STEP 0x1A
#define _OV5640_CMD_AF_GET_VCM_STEP 0x1B
#define _OV5640_CMD_MAIN 0x3022
#define _OV5640_CMD_ACK 0x3023
#define _OV5640_CMD_PARA0 0x3024
#define _OV5640_CMD_PARA1 0x3025
#define _OV5640_CMD_PARA2 0x3026
#define _OV5640_CMD_PARA3 0x3027
#define _OV5640_CMD_PARA4 0x3028
#define _OV5640_CMD_FW_STATUS 0x3029
