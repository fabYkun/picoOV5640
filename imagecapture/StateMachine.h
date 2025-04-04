#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

enum { PIO_ANY_OFFSET = -1 };
enum { PIO_FIFO_JOIN_AUTO = -1, PIO_FIFO_TYPE_DEFAULT = PIO_FIFO_JOIN_AUTO };
enum { PIO_MOV_STATUS_DEFAULT = STATUS_TX_LESSTHAN };
enum { PIO_MOV_N_DEFAULT = 0 };

#define RP2PIO_STATEMACHINE_N_BUFS 3

typedef struct { uint32_t value32;
} pio_pinmask32_t;
typedef struct { uint64_t value;
} pio_pinmask_t;

typedef struct _mp_obj_type_t mp_obj_type_t;

struct _mp_obj_base_t {
    const mp_obj_type_t *type;
};
typedef struct _mp_obj_base_t mp_obj_base_t;

typedef void *mp_obj_t;

struct _mp_obj_type_t {
    mp_obj_base_t base;
    uint16_t flags;
    uint16_t name;
    uint8_t slot_index_make_new;
    uint8_t slot_index_print;
    uint8_t slot_index_call;
    uint8_t slot_index_unary_op;
    uint8_t slot_index_binary_op;
    uint8_t slot_index_attr;
    uint8_t slot_index_subscr;
    uint8_t slot_index_iter;
    uint8_t slot_index_buffer;
    uint8_t slot_index_protocol;
    uint8_t slot_index_parent;
    uint8_t slot_index_locals_dict;
    const void *slots[];
};

typedef struct _mp_buffer_info_t {
    void *buf;      // can be NULL if len == 0
    size_t len;     // in bytes
    int typecode;   // as per binary.h
} mp_buffer_info_t;

typedef struct sm_buf_info {
    mp_obj_t obj;
    mp_buffer_info_t info;
} sm_buf_info;

typedef enum { SRAM, ROM, XIP, IO } memorymap_rp2_section_t;

typedef struct {
    mp_obj_base_t base;
    uint8_t *start_address;
    size_t len;
    memorymap_rp2_section_t type;
} memorymap_addressrange_obj_t;

typedef struct {
    mp_obj_base_t base;
    pio_pinmask32_t pins; // Bitmask of what pins this state machine uses.
    int state_machine;
    PIO pio;
    const uint16_t *init;
    size_t init_len;
    pio_pinmask_t initial_pin_state;
    pio_pinmask_t initial_pin_direction;
    pio_pinmask_t pull_pin_up;
    pio_pinmask_t pull_pin_down;
    uint tx_dreq;
    uint rx_dreq;
    uint32_t actual_frequency;
    pio_sm_config sm_config;
    bool in;
    bool out;
    bool wait_for_txstall;
    bool out_shift_right;
    bool in_shift_right;
    bool user_interruptible;
    #if NUM_BANK0_GPIOS > 32
    uint8_t pio_gpio_offset;
    #endif
    uint8_t offset;
    uint8_t fifo_depth;  // Either 4 if FIFOs are not joined, or 8 if they are.

    // dma-related items
    volatile int pending_buffers_write;
    volatile int pending_buffers_read;
    int write_buf_index, read_buf_index;
    sm_buf_info write_buf[RP2PIO_STATEMACHINE_N_BUFS];
    sm_buf_info read_buf[RP2PIO_STATEMACHINE_N_BUFS];

    sm_buf_info once_read_buf_info, loop_read_buf_info, loop2_read_buf_info;
    sm_buf_info current_read_buf, next_read_buf_1, next_read_buf_2, next_read_buf_3;
    sm_buf_info once_write_buf_info, loop_write_buf_info, loop2_write_buf_info;
    sm_buf_info current_write_buf, next_write_buf_1, next_write_buf_2, next_write_buf_3;

    bool switched_write_buffers, switched_read_buffers;

    int background_stride_in_bytes;
    bool dma_completed_write, byteswap;
    bool dma_completed_read;
    #if PICO_PIO_VERSION > 0
    memorymap_addressrange_obj_t rxfifo_obj;
    #endif
} rp2pio_statemachine_obj_t;

typedef struct {
    uint8_t number;
} mcu_pin_obj_t;

typedef enum _digitalio_pull_t {
    PULL_NONE,
    PULL_UP,
    PULL_DOWN
} digitalio_pull_t;

typedef struct {
    struct {
        pio_pinmask_t pins_we_use;
        uint8_t in_pin_count, out_pin_count, pio_gpio_offset;
        bool has_jmp_pin, auto_push, auto_pull, has_in_pin, has_out_pin, has_set_pin;
    } inputs;
    struct {
        bool tx_fifo, rx_fifo, in_loaded, out_loaded, in_used, out_used;
    } outputs;
} introspect_t;

typedef uint64_t pio_pinmask_value_t;

bool rp2pio_statemachine_construct(rp2pio_statemachine_obj_t *self,
    const uint16_t *program, size_t program_len,
    size_t frequency,
    const uint16_t *init, size_t init_len,
    const mcu_pin_obj_t *first_out_pin, uint8_t out_pin_count,
    const mcu_pin_obj_t *first_in_pin, uint8_t in_pin_count,
    pio_pinmask_t pull_pin_up, pio_pinmask_t pull_pin_down,
    const mcu_pin_obj_t *first_set_pin, uint8_t set_pin_count,
    const mcu_pin_obj_t *first_sideset_pin, uint8_t sideset_pin_count, bool sideset_pindirs,
    pio_pinmask_t initial_pin_state, pio_pinmask_t initial_pin_direction,
    const mcu_pin_obj_t *jmp_pin,
    pio_pinmask_t pins_we_use, bool tx_fifo, bool rx_fifo,
    bool auto_pull, uint8_t pull_threshold, bool out_shift_right,
    bool wait_for_txstall,
    bool auto_push, uint8_t push_threshold, bool in_shift_right,
    bool claim_pins,
    bool interruptible,
    bool sideset_enable,
    int wrap_target, int wrap, int offset,
    int fifo_type,
    int mov_status_type, int mov_status_n
    );

#define PIO_PINMASK(i) (UINT64_C(1) << (i))
#define PIO_PINMASK_VALUE(p) ((p).value)
#define PIO_PINMASK_FROM_VALUE(v) ((pio_pinmask_t) {(v)})
#define PIO_PINMASK_FROM_PIN(i) ((pio_pinmask_t) {(PIO_PINMASK(i))})
#define PIO_PINMASK_NONE PIO_PINMASK_FROM_VALUE(0)
#define PIO_PINMASK_SET(p, i) ((p).value |= PIO_PINMASK(i))
#define PIO_PINMASK_CLEAR(p, i) ((p).value &= ~PIO_PINMASK(i))
#define PIO_PINMASK_IS_SET(p, i) (((p).value & PIO_PINMASK(i)) != 0)
#define PIO_PINMASK_BINOP(op, p, q) PIO_PINMASK_FROM_VALUE((p).value op(q).value)
#define PIO_PINMASK_BINOP_ASSIGN(op, p, q) ((p).value op(q).value)
#define PIO_PINMASK_EQUAL(p, q) ((p).value == (q).value)
#define PIO_PINMASK_AND(p, q) PIO_PINMASK_BINOP(&, (p), (q))
#define PIO_PINMASK_AND_NOT(p, q) PIO_PINMASK_BINOP(&~, (p), (q))
#define PIO_PINMASK_OR(p, q) PIO_PINMASK_BINOP(|, (p), (q))
#define PIO_PINMASK_OR3(p, q, r) PIO_PINMASK_OR((p), PIO_PINMASK_OR((q), (r)))
#define PIO_PINMASK_INTERSECT(p, q) PIO_PINMASK_BINOP_ASSIGN( &=, (p), (q))
#define PIO_PINMASK_DIFFERENCE(p, q) PIO_PINMASK_BINOP_ASSIGN( &= ~, (p), (q))
#define PIO_PINMASK_MERGE(p, q) PIO_PINMASK_BINOP_ASSIGN( |=, (p), (q))

#define PIO_PINMASK32(i) (1u << (i))
#define PIO_PINMASK32_C(c) UINT32_C(c)
#define PIO_PINMASK32_NONE PIO_PINMASK32_FROM_VALUE(0)
#define PIO_PINMASK32_ALL PIO_PINMASK32_FROM_VALUE(~UINT32_C(0))
#define PIO_PINMASK32_BASE(i, base) PIO_PINMASK32((i) - (base))
#define PIO_PINMASK32_VALUE(p) ((p).value32)
#define PIO_PINMASK32_FROM_VALUE(v) ((pio_pinmask32_t) {(v)})
#define PIO_PINMASK32_SET(p, i) ((p).value32 |= PIO_PINMASK32_VALUE(i))
#define PIO_PINMASK32_CLEAR(p, i) ((p).value32 &= ~PIO_PINMASK32_VALUE(i))
#define PIO_PINMASK32_IS_SET(p, i) (((p).value32 & PIO_PINMASK32_VALUE(i)) != 0)
#define PIO_PINMASK32_BINOP(op, p, q) PIO_PINMASK32_FROM_VALUE((p).value32 op(q).value32)
#define PIO_PINMASK32_AND(p, q) PIO_PINMASK32_BINOP(&, (p), (q))
#define PIO_PINMASK32_AND_NOT(p, q) PIO_PINMASK32_BINOP(&~, (p), (q))
#define PIO_PINMASK32_OR(p, q) PIO_PINMASK32_BINOP(|, (p), (q))
#define PIO_PINMASK32_OR3(p, q, r) PIO_PINMASK32_OR((p), PIO_PINMASK32_OR((q), (r)))
#define PIO_PINMASK32_INTERSECT(p, q) PIO_PINMASK32_BINOP( &=, (p), (q))
#define PIO_PINMASK32_DIFFERENCE(p, q) PIO_PINMASK32_BINOP( &= ~, (p), (q))
#define PIO_PINMASK32_MERGE(p, q) PIO_PINMASK32_BINOP( |=, (p), (q))
#define PIO_PINMASK32_FROM_PINMASK_WITH_OFFSET(p, gpio_offset) PIO_PINMASK32_FROM_VALUE(PIO_PINMASK_VALUE((p)) >> (gpio_offset))
#define PIO_PINMASK_FROM_PINMASK32_WITH_OFFSET(p, gpio_offset) PIO_PINMASK_FROM_VALUE(PIO_PINMASK32_VALUE((p)) << (gpio_offset))
#define PIO_PINMASK_C(c) UINT64_C(c)

#define MP_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

void common_hal_rp2pio_statemachine_construct(rp2pio_statemachine_obj_t *self,
    const uint16_t *program, size_t program_len,
    size_t frequency,
    const uint16_t *init, size_t init_len,
    const uint16_t *may_exec, size_t may_exec_len,
    const mcu_pin_obj_t *first_out_pin, uint8_t out_pin_count, pio_pinmask32_t initial_out_pin_state, pio_pinmask32_t initial_out_pin_direction,
    const mcu_pin_obj_t *first_in_pin, uint8_t in_pin_count, pio_pinmask32_t in_pull_pin_up, pio_pinmask32_t in_pull_pin_down,
    const mcu_pin_obj_t *first_set_pin, uint8_t set_pin_count, pio_pinmask32_t initial_set_pin_state, pio_pinmask32_t initial_set_pin_direction,
    const mcu_pin_obj_t *first_sideset_pin, uint8_t sideset_pin_count, bool sideset_pindirs,
    pio_pinmask32_t initial_sideset_pin_state, pio_pinmask32_t initial_sideset_pin_direction,
    bool sideset_enable,
    const mcu_pin_obj_t *jmp_pin, digitalio_pull_t jmp_pin_pull,
    pio_pinmask_t wait_gpio_mask,
    bool exclusive_pin_use,
    bool auto_pull, uint8_t pull_threshold, bool out_shift_right,
    bool wait_for_txstall,
    bool auto_push, uint8_t push_threshold, bool in_shift_right,
    bool user_interruptible,
    int wrap_target, int wrap,
    int offset,
    int fifo_type,
    int mov_status_type,
    int mov_status_n);

bool transfer_in(rp2pio_statemachine_obj_t *self, uint8_t *data_in, size_t len, uint8_t in_stride_in_bytes, bool swap_in);

uint8_t rp2pio_statemachine_program_offset(rp2pio_statemachine_obj_t *self);