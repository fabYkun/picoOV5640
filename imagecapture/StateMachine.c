#include "StateMachine.h"
#include <stdio.h>

static pio_pinmask_t _current_pins[NUM_PIOS];
static pio_pinmask_t _current_sm_pins[NUM_PIOS][NUM_PIO_STATE_MACHINES];

static uint32_t _current_program_id[NUM_PIOS][NUM_PIO_STATE_MACHINES];
static uint8_t _current_program_offset[NUM_PIOS][NUM_PIO_STATE_MACHINES];
static uint8_t _current_program_len[NUM_PIOS][NUM_PIO_STATE_MACHINES];

static uint8_t _pin_reference_count[NUM_BANK0_GPIOS];

static int8_t _sm_dma_plus_one_write[NUM_PIOS][NUM_PIO_STATE_MACHINES];
static int8_t _sm_dma_plus_one_read[NUM_PIOS][NUM_PIO_STATE_MACHINES];

#define SM_DMA_ALLOCATED_WRITE(pio_index, sm) (_sm_dma_plus_one_write[(pio_index)][(sm)] != 0)
#define SM_DMA_GET_CHANNEL_WRITE(pio_index, sm) (_sm_dma_plus_one_write[(pio_index)][(sm)] - 1)
#define SM_DMA_CLEAR_CHANNEL_WRITE(pio_index, sm) (_sm_dma_plus_one_write[(pio_index)][(sm)] = 0)
#define SM_DMA_SET_CHANNEL_WRITE(pio_index, sm, channel) (_sm_dma_plus_one_write[(pio_index)][(sm)] = (channel) + 1)

#define SM_DMA_ALLOCATED_READ(pio_index, sm) (_sm_dma_plus_one_read[(pio_index)][(sm)] != 0)
#define SM_DMA_GET_CHANNEL_READ(pio_index, sm) (_sm_dma_plus_one_read[(pio_index)][(sm)] - 1)
#define SM_DMA_CLEAR_CHANNEL_READ(pio_index, sm) (_sm_dma_plus_one_read[(pio_index)][(sm)] = 0)
#define SM_DMA_SET_CHANNEL_READ(pio_index, sm, channel) (_sm_dma_plus_one_read[(pio_index)][(sm)] = (channel) + 1)

static uint64_t gpio_bank0_pin_claimed;

const mcu_pin_obj_t _pins[] = {
    [0] = { .number = 0 },
    [1] = { .number = 1 },
    [2] = { .number = 2 },
    [3] = { .number = 3 },
    [4] = { .number = 4 },
    [5] = { .number = 5 },
    [6] = { .number = 6 },
    [7] = { .number = 7 },
    [8] = { .number = 8 },
    [9] = { .number = 9 },
    [10] = { .number = 10 },
    [11] = { .number = 11 },
    [12] = { .number = 12 },
    [13] = { .number = 13 },
    [14] = { .number = 14 },
    [15] = { .number = 15 },
    [16] = { .number = 16 },
    [17] = { .number = 17 },
    [18] = { .number = 18 },
    [19] = { .number = 19 },
    [20] = { .number = 20 },
    [21] = { .number = 21 },
    [22] = { .number = 22 },
    [23] = { .number = 137 },
    [24] = { .number = 137 },
    [25] = { .number = 137 },
    [26] = { .number = 26 },
    [27] = { .number = 27 },
    [28] = { .number = 28 },
    [29] = { .number = 29 },
};

const mcu_pin_obj_t *mcu_get_pin_by_number(uint8_t number) {
    if (number < NUM_BANK0_GPIOS)
        return &_pins[number];
    return NULL;
}

void claim_pin(const mcu_pin_obj_t *pin) {
    if (pin->number >= NUM_BANK0_GPIOS)
        return;
    gpio_bank0_pin_claimed |= (1LL << pin->number);
}

void mp_arg_validate_int_range(int i, int min, int max) {
    if (i < min || i > max)
        printf("(arg_validate_int_range): program must be %d-%d\n", min, max);
}

static enum pio_fifo_join compute_fifo_type(int fifo_type_in, bool rx_fifo, bool tx_fifo) {
    if (fifo_type_in != PIO_FIFO_JOIN_AUTO) {
        return fifo_type_in;
    } else if (!rx_fifo) {
        return PIO_FIFO_JOIN_TX;
    } else if (!tx_fifo) {
        return PIO_FIFO_JOIN_RX;
    }
    return PIO_FIFO_JOIN_NONE;
}

static int compute_fifo_depth(enum pio_fifo_join join) {
    #if PICO_PIO_VERSION > 0
    if (join == PIO_FIFO_JOIN_PUTGET)
        return 0;
    #endif
    if (join == PIO_FIFO_JOIN_TX || join == PIO_FIFO_JOIN_RX) {
        return 8;
    }
    return 4;
}

uint8_t rp2pio_statemachine_program_offset(rp2pio_statemachine_obj_t *self) {
    uint8_t pio_index = pio_get_index(self->pio);
    uint8_t sm = self->state_machine;
    return _current_program_offset[pio_index][sm];
}

static void consider_instruction(introspect_t *state, uint16_t full_instruction, size_t i) {
    uint16_t instruction = full_instruction & 0xe000;
    if (instruction == 0x8000) {
        if ((full_instruction & 0xe080) == pio_instr_bits_push) {
            state->outputs.rx_fifo = true;
            state->outputs.in_loaded = true;
        } else { // pull otherwise.
            state->outputs.tx_fifo = true;
            state->outputs.out_loaded = true;
        }
    }
    if (instruction == pio_instr_bits_jmp) {
        uint16_t condition = (full_instruction & 0x00e0) >> 5;
        if ((condition == 0x6) && !state->inputs.has_jmp_pin) {
            printf("Missing jmp_pin. [%u] jumps on pin\n", i);
        }
    }
    if (instruction == pio_instr_bits_wait) {
        uint16_t wait_source = (full_instruction & 0x0060) >> 5;
        uint16_t wait_index = (full_instruction & 0x001f) + state->inputs.pio_gpio_offset;
        if (wait_source == 0 && !PIO_PINMASK_IS_SET(state->inputs.pins_we_use, wait_index)) { // GPIO
            printf("program[%u] uses extra pin\n", i);
        } else if (wait_source == 1) { // Input pin
            if (!state->inputs.has_in_pin) {
                printf("Missing first_in_pin. program[%u] waits based on pin\n", i);
            }
            if (wait_index >= state->inputs.in_pin_count) {
                printf("program[%u] waits on input outside of count\n", i);
            }
        }
    }
    if (instruction == pio_instr_bits_in) {
        uint16_t source = (full_instruction & 0x00e0) >> 5;
        uint16_t bit_count = full_instruction & 0x001f;
        if (source == 0) {
            if (!state->inputs.has_in_pin) {
                printf("Missing first_in_pin. program[%u] shifts in from pin(s)\n", i);
            }
            if (bit_count > state->inputs.in_pin_count) {
                printf("program[%u] shifts in more bits than pin count\n", i);
            }
        }
        if (state->inputs.auto_push) {
            state->outputs.in_loaded = true;
            state->outputs.rx_fifo = true;
        }
        state->outputs.in_used = true;
    }
    if (instruction == pio_instr_bits_out) {
        uint16_t bit_count = full_instruction & 0x001f;
        uint16_t destination = (full_instruction & 0x00e0) >> 5;
        // Check for pins or pindirs destination.
        if (destination == 0x0 || destination == 0x4) {
            if (!state->inputs.has_out_pin) {
                printf("Missing first_out_pin. program[%u] shifts out to pin(s)\n", i);
            }
            if (bit_count > state->inputs.out_pin_count) {
                printf("program[%u] shifts out more bits than pin count\n", i);
            }
        }
        if (state->inputs.auto_pull) {
            state->outputs.out_loaded = true;
            state->outputs.tx_fifo = true;
        }
        state->outputs.out_used = true;
    }
    if (instruction == pio_instr_bits_set) {
        uint16_t destination = (full_instruction & 0x00e0) >> 5;
        // Check for pins or pindirs destination.
        if ((destination == 0x00 || destination == 0x4) && !state->inputs.has_set_pin) {
            printf("Missing first_set_pin. program[%u] sets pin(s)\n", i);
        }
    }
    if (instruction == pio_instr_bits_mov) {
        uint16_t source = full_instruction & 0x0007;
        uint16_t destination = (full_instruction & 0x00e0) >> 5;
        // Check for pins or pindirs destination.
        if (destination == 0x0 && !state->inputs.has_out_pin) {
            printf("Missing first_out_pin. program[%u] writes pin(s)\n", i);
        }
        if (source == 0x0 && !state->inputs.has_in_pin) {
            printf("Missing first_in_pin. program[%u] reads pin(s)\n", i);
        }
        if (destination == 0x6) {
            state->outputs.in_loaded = true;
        } else if (destination == 0x7) {
            state->outputs.out_loaded = true;
        }
    }
}

static void consider_program(introspect_t *state, const uint16_t *program, size_t program_len) {
    for (size_t i = 0; i < program_len; i++) {
        consider_instruction(state, program[i], i);
    }
}

static void rp2pio_statemachine_set_pull(pio_pinmask_t pull_pin_up, pio_pinmask_t pull_pin_down, pio_pinmask_t pins_we_use) {
    for (size_t i = 0; i < NUM_BANK0_GPIOS; i++) {
        bool used = PIO_PINMASK_IS_SET(pins_we_use, i);
        if (used) {
            bool pull_up = PIO_PINMASK_IS_SET(pull_pin_up, i);
            bool pull_down = PIO_PINMASK_IS_SET(pull_pin_down, i);
            gpio_set_pulls(i, pull_up, pull_down);
        }
    }
}

static pio_pinmask_t _check_pins_free(const mcu_pin_obj_t *first_pin, uint8_t pin_count, bool exclusive_pin_use) {
    pio_pinmask_t pins_we_use = PIO_PINMASK_NONE;
    if (first_pin != NULL) {
        for (size_t i = 0; i < pin_count; i++) {
            uint8_t pin_number = first_pin->number + i;
            if (pin_number >= NUM_BANK0_GPIOS) {
                printf("Pin count too large\n");
            }
            const mcu_pin_obj_t *pin = mcu_get_pin_by_number(pin_number);
            if (!pin || (exclusive_pin_use || _pin_reference_count[pin_number] == 0)) {
                printf("pin in use\n");
            }
            PIO_PINMASK_SET(pins_we_use, pin_number);
        }
    }
    return pins_we_use;
}

static enum dma_channel_transfer_size _stride_to_dma_size(uint8_t stride) {
    switch (stride) {
        case 4:
            return DMA_SIZE_32;
        case 2:
            return DMA_SIZE_16;
        case 1:
        default:
            return DMA_SIZE_8;
    }
}

bool transfer_in(rp2pio_statemachine_obj_t *self,
    uint8_t *data_in, size_t len, uint8_t in_stride_in_bytes, bool swap_in) {
    // This implementation is based on SPI

    // Use DMA for large transfers if channels are available.
    // Don't exceed FIFO size.
    const size_t dma_min_size_threshold = self->fifo_depth;
    int chan_rx = -1;
    bool use_dma = len >= dma_min_size_threshold || swap_in;
    if (use_dma) {
        chan_rx = dma_claim_unused_channel(true);
        if (chan_rx < 0) {
            printf("DMA allocation failed\n");
            return false;
        }
    }
    const volatile uint8_t *rx_source = (const volatile uint8_t *)&self->pio->rxf[self->state_machine];
    if (self->in_shift_right) {
        rx_source += 4 - in_stride_in_bytes;
    }
    uint32_t stall_mask = 1 << (PIO_FDEBUG_TXSTALL_LSB + self->state_machine);
    if (use_dma) {
        dma_channel_config c;
        uint32_t channel_mask = 0;
        {
            c = dma_channel_get_default_config(chan_rx);
            channel_config_set_transfer_data_size(&c, _stride_to_dma_size(in_stride_in_bytes));
            channel_config_set_dreq(&c, self->rx_dreq);
            channel_config_set_read_increment(&c, false);
            channel_config_set_write_increment(&c, true);
            channel_config_set_bswap(&c, swap_in);
            dma_channel_configure(chan_rx, &c,
                data_in,
                rx_source,
                len / in_stride_in_bytes,
                false);
            channel_mask |= 1u << chan_rx;
        }

        dma_start_channel_mask(channel_mask);
        while (dma_channel_is_busy(chan_rx)) {
            sleep_ms(10);
        }
        // Clear the stall bit so we can detect when the state machine is done transmitting.
        self->pio->fdebug = stall_mask;
    }

    // If we have claimed only one channel successfully, we should release immediately. This also
    // releases the DMA after use_dma has been done.
    if (chan_rx >= 0) {
        dma_channel_unclaim(chan_rx);
    }

    if (!use_dma) {
        // Use software for small transfers, or if couldn't claim two DMA channels
        size_t rx_remaining = len / in_stride_in_bytes;

        while (rx_remaining) {
            while (rx_remaining && !pio_sm_is_rx_fifo_empty(self->pio, self->state_machine)) {
                if (in_stride_in_bytes == 1) {
                    *data_in = (uint8_t)*rx_source;
                } else if (in_stride_in_bytes == 2) {
                    *((uint16_t *)data_in) = *((uint16_t *)rx_source);
                } else if (in_stride_in_bytes == 4) {
                    *((uint32_t *)data_in) = *((uint32_t *)rx_source);
                }
                data_in += in_stride_in_bytes;
                --rx_remaining;
            }
        }
        // Clear the stall bit so we can detect when the state machine is done transmitting.
        self->pio->fdebug = stall_mask;
    }
    return true;
}

static bool is_gpio_compatible(PIO pio, uint32_t used_gpio_ranges) {
    #if PICO_PIO_VERSION > 0
    bool gpio_base = pio_get_gpio_base(pio);
    return !((gpio_base && (used_gpio_ranges & 1)) ||
        (!gpio_base && (used_gpio_ranges & 4)));
    #else
    ((void)pio);
    ((void)used_gpio_ranges);
    return true;
    #endif
}

static pio_pinmask_t mask_and_shift(const mcu_pin_obj_t *first_pin, uint32_t bit_count, pio_pinmask32_t value_in) {
    if (!first_pin) {
        return PIO_PINMASK_NONE;
    }
    pio_pinmask_value_t mask = (PIO_PINMASK_C(1) << bit_count) - 1;
    pio_pinmask_value_t value = (pio_pinmask_value_t)PIO_PINMASK32_VALUE(value_in);
    int shift = first_pin->number;
    return PIO_PINMASK_FROM_VALUE((value & mask) << shift);
}

static bool use_existing_program(PIO *pio_out, uint *sm_out, int *offset_inout, uint32_t program_id, size_t program_len, uint gpio_base, uint gpio_count) {
    uint32_t required_gpio_ranges;
    if (gpio_count) {
        required_gpio_ranges = (1u << (gpio_base >> 4)) |
            (1u << ((gpio_base + gpio_count - 1) >> 4));
    } else {
        required_gpio_ranges = 0;
    }

    for (size_t i = 0; i < NUM_PIOS; i++) {
        PIO pio = pio_get_instance(i);
        if (!is_gpio_compatible(pio, required_gpio_ranges)) {
            continue;
        }
        for (size_t j = 0; j < NUM_PIO_STATE_MACHINES; j++) {
            if (_current_program_id[i][j] == program_id &&
                _current_program_len[i][j] == program_len &&
                (*offset_inout == -1 || *offset_inout == _current_program_offset[i][j])) {
                *sm_out = pio_claim_unused_sm(pio, false);
                if (*sm_out >= 0) {
                    *pio_out = pio;
                    *offset_inout = _current_program_offset[i][j];
                    return true;
                }
            }
        }
    }
    return false;
}

void common_hal_rp2pio_statemachine_construct(rp2pio_statemachine_obj_t *self,
    const uint16_t *program, size_t program_len,
    size_t frequency,
    const uint16_t *init, size_t init_len,
    const uint16_t *may_exec, size_t may_exec_len,
    const mcu_pin_obj_t *first_out_pin, uint8_t out_pin_count, pio_pinmask32_t initial_out_pin_state32, pio_pinmask32_t initial_out_pin_direction32,
    const mcu_pin_obj_t *first_in_pin, uint8_t in_pin_count,
    pio_pinmask32_t in_pull_pin_up32, pio_pinmask32_t in_pull_pin_down32, // relative to first_in_pin
    const mcu_pin_obj_t *first_set_pin, uint8_t set_pin_count, pio_pinmask32_t initial_set_pin_state32, pio_pinmask32_t initial_set_pin_direction32,
    const mcu_pin_obj_t *first_sideset_pin, uint8_t sideset_pin_count, bool sideset_pindirs,
    pio_pinmask32_t initial_sideset_pin_state32, pio_pinmask32_t initial_sideset_pin_direction32,
    bool sideset_enable,
    const mcu_pin_obj_t *jmp_pin, digitalio_pull_t jmp_pull,
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
    int mov_status_n) {

    // First, check that all pins are free OR already in use by any PIO if exclusive_pin_use is false.
    pio_pinmask_t pins_we_use = wait_gpio_mask;
    PIO_PINMASK_MERGE(pins_we_use, _check_pins_free(first_out_pin, out_pin_count, exclusive_pin_use));
    PIO_PINMASK_MERGE(pins_we_use, _check_pins_free(first_in_pin, in_pin_count, exclusive_pin_use));
    PIO_PINMASK_MERGE(pins_we_use, _check_pins_free(first_set_pin, set_pin_count, exclusive_pin_use));
    PIO_PINMASK_MERGE(pins_we_use, _check_pins_free(first_sideset_pin, sideset_pin_count, exclusive_pin_use));
    PIO_PINMASK_MERGE(pins_we_use, _check_pins_free(jmp_pin, 1, exclusive_pin_use));

    int pio_gpio_offset = 0;
    #if NUM_BANK0_GPIOS > 32
    if (PIO_PINMASK_VALUE(pins_we_use) >> 32) {
        pio_gpio_offset = 16;
        if (PIO_PINMASK_VALUE(pins_we_use) & 0xffff) {
            printf("Cannot use GPIO0..15 together with GPIO32..47");
        }
    }
    #endif

    // Look through the program to see what we reference and make sure it was provided.
    introspect_t state = {
        .inputs = {
            .pins_we_use = pins_we_use,
            .pio_gpio_offset = pio_gpio_offset,
            .has_jmp_pin = jmp_pin != NULL,
            .has_in_pin = first_in_pin != NULL,
            .has_out_pin = first_out_pin != NULL,
            .has_set_pin = first_set_pin != NULL,
            .in_pin_count = in_pin_count,
            .out_pin_count = out_pin_count,
            .auto_pull = auto_pull,
            .auto_push = auto_push,
        }
    };
    consider_program(&state, program, program_len);
    consider_program(&state, init, init_len);
    consider_program(&state, may_exec, may_exec_len);

    if (!state.outputs.in_loaded && state.outputs.in_used) {
        printf("Program does IN without loading ISR\n");
    }
    if (!state.outputs.out_loaded && state.outputs.out_used) {
        printf("Program does OUT without loading OSR\n");
    }

    pio_pinmask_t initial_pin_state = mask_and_shift(first_out_pin, out_pin_count, initial_out_pin_state32);
    pio_pinmask_t initial_pin_direction = mask_and_shift(first_out_pin, out_pin_count, initial_out_pin_direction32);
    pio_pinmask_t initial_set_pin_state = mask_and_shift(first_set_pin, set_pin_count, initial_set_pin_state32);
    pio_pinmask_t initial_set_pin_direction = mask_and_shift(first_set_pin, set_pin_count, initial_set_pin_direction32);
    pio_pinmask_t set_out_overlap = PIO_PINMASK_AND(mask_and_shift(first_out_pin, out_pin_count, PIO_PINMASK32_ALL),
        mask_and_shift(first_set_pin, set_pin_count, PIO_PINMASK32_ALL));
    // Check that OUT and SET settings agree because we don't have a way of picking one over the other.
    if (!PIO_PINMASK_EQUAL(
        PIO_PINMASK_AND(initial_pin_state, set_out_overlap),
        PIO_PINMASK_AND(initial_set_pin_state, set_out_overlap))) {
            printf("Initial set pin state conflicts with initial out pin state\n");
    }
    if (!PIO_PINMASK_EQUAL(
        PIO_PINMASK_AND(initial_pin_direction, set_out_overlap),
        PIO_PINMASK_AND(initial_set_pin_direction, set_out_overlap))) {
            printf("Initial set pin direction conflicts with initial out pin direction\n");
    }
    PIO_PINMASK_MERGE(initial_pin_state, initial_set_pin_state);
    PIO_PINMASK_MERGE(initial_pin_direction, initial_set_pin_direction);

    // Sideset overrides OUT or SET so we always use its values.
    pio_pinmask_t sideset_mask = mask_and_shift(first_sideset_pin, sideset_pin_count, PIO_PINMASK32_FROM_VALUE(0x1f));
    initial_pin_state = PIO_PINMASK_OR(
        PIO_PINMASK_AND_NOT(initial_pin_state, sideset_mask),
        mask_and_shift(first_sideset_pin, sideset_pin_count, initial_sideset_pin_state32));
    initial_pin_direction = PIO_PINMASK_OR(
        PIO_PINMASK_AND_NOT(initial_pin_direction, sideset_mask),
        mask_and_shift(first_sideset_pin, sideset_pin_count, initial_sideset_pin_direction32));

    // Deal with pull up/downs
    pio_pinmask_t pull_up = mask_and_shift(first_in_pin, in_pin_count, in_pull_pin_up32);
    pio_pinmask_t pull_down = mask_and_shift(first_in_pin, in_pin_count, in_pull_pin_down32);

    if (jmp_pin) {
        pio_pinmask_t jmp_mask = mask_and_shift(jmp_pin, 1, PIO_PINMASK32_FROM_VALUE(0x1f));
        if (jmp_pull == PULL_UP) {
            PIO_PINMASK_MERGE(pull_up, jmp_mask);
        }
        if (jmp_pull == PULL_DOWN) {
            PIO_PINMASK_MERGE(pull_down, jmp_mask);
        }
    }
    if (PIO_PINMASK_VALUE(
        PIO_PINMASK_AND(initial_pin_direction,
            PIO_PINMASK_OR(pull_up, pull_down)))) {
        printf("pull masks conflict with direction masks\n");
    }

    bool ok = rp2pio_statemachine_construct(
        self,
        program, program_len,
        frequency,
        init, init_len,
        first_out_pin, out_pin_count,
        first_in_pin, in_pin_count,
        pull_up, pull_down,
        first_set_pin, set_pin_count,
        first_sideset_pin, sideset_pin_count, sideset_pindirs,
        initial_pin_state, initial_pin_direction,
        jmp_pin,
        pins_we_use, state.outputs.tx_fifo, state.outputs.rx_fifo,
        auto_pull, pull_threshold, out_shift_right,
        wait_for_txstall,
        auto_push, push_threshold, in_shift_right,
        true /* claim pins */,
        user_interruptible,
        sideset_enable,
        wrap_target, wrap, offset,
        fifo_type,
        mov_status_type, mov_status_n);
    if (!ok) {
        printf("All state machines in use\n");
    }
}

void common_hal_rp2pio_statemachine_set_frequency(rp2pio_statemachine_obj_t *self, uint32_t frequency, uint64_t frequency256, uint64_t div256) {
    // 0 is interpreted as 0x10000 so it's valid.
    if (div256 / 256 > 0x10000 || frequency > clock_get_hz(clk_sys)) {
        printf("frequence out of range\n");
    }
    self->actual_frequency = frequency256 / div256;

    pio_sm_set_clkdiv_int_frac(self->pio, self->state_machine, div256 / 256, div256 % 256);
    // Reset the clkdiv counter in case our new TOP is lower.
    pio_sm_clkdiv_restart(self->pio, self->state_machine);
}

void common_hal_rp2pio_statemachine_run(rp2pio_statemachine_obj_t *self, const uint16_t *instructions, size_t len) {
    for (size_t i = 0; i < len; i++) {
        pio_sm_exec(self->pio, self->state_machine, instructions[i]);
    }
}

bool rp2pio_statemachine_construct(rp2pio_statemachine_obj_t *self,
    const uint16_t *program, size_t program_len,
    size_t frequency,
    const uint16_t *init, size_t init_len,
    const mcu_pin_obj_t *first_out_pin, uint8_t out_pin_count,
    const mcu_pin_obj_t *first_in_pin, uint8_t in_pin_count,
    pio_pinmask_t pull_pin_up, pio_pinmask_t pull_pin_down, // GPIO numbering
    const mcu_pin_obj_t *first_set_pin, uint8_t set_pin_count,
    const mcu_pin_obj_t *first_sideset_pin, uint8_t sideset_pin_count, bool sideset_pindirs,
    pio_pinmask_t initial_pin_state, pio_pinmask_t initial_pin_direction,
    const mcu_pin_obj_t *jmp_pin,
    pio_pinmask_t pins_we_use, bool tx_fifo, bool rx_fifo,
    bool auto_pull, uint8_t pull_threshold, bool out_shift_right,
    bool wait_for_txstall,
    bool auto_push, uint8_t push_threshold, bool in_shift_right,
    bool claim_pins,
    bool user_interruptible,
    bool sideset_enable,
    int wrap_target, int wrap,
    int offset,
    int fifo_type,
    int mov_status_type, int mov_status_n
    ) {
    // Create a program id that isn't the pointer so we can store it without storing the original object.
    uint32_t program_id = ~((uint32_t)program);

    uint gpio_base = 0, gpio_count = 0;
    #if NUM_BANK0_GPIOS > 32
    if (PIO_PINMASK_VALUE(pins_we_use) >> 32) {
        if (PIO_PINMASK_VALUE(pins_we_use) & 0xffff) {
            // Uses pins from 0-15 and 32-47. not possible
            return false;
        }
    }

    pio_pinmask_value_t v = PIO_PINMASK_VALUE(pins_we_use);
    if (v) {
        while (!(v & 1)) {
            gpio_base++;
            v >>= 1;
        }
        while (v) {
            gpio_count++;
            v >>= 1;
        }
    }
    #endif

    // Next, find a PIO and state machine to use.
    pio_program_t program_struct = {
        .instructions = (uint16_t *)program,
        .length = program_len,
        .origin = offset,
    };
    PIO pio;
    uint state_machine;
    bool added = false;

    if (!use_existing_program(&pio, &state_machine, &offset, program_id, program_len, gpio_base, gpio_count)) {
        uint program_offset;
        bool r = pio_claim_free_sm_and_add_program_for_gpio_range(&program_struct, &pio, &state_machine, &program_offset, gpio_base, gpio_count, true);
        if (!r) {
            return false;
        }
        offset = program_offset;
        added = true;
    }

    size_t pio_index = pio_get_index(pio);
    for (size_t i = 0; i < NUM_PIOS; i++) {
        if (i == pio_index) {
            continue;
        }
        pio_pinmask_t intersection = PIO_PINMASK_AND(_current_pins[i], pins_we_use);
        if (PIO_PINMASK_VALUE(intersection) != 0) {
            if (added) {
                pio_remove_program(pio, &program_struct, offset);
            }
            pio_sm_unclaim(pio, state_machine);
            // Pin in use by another PIO already.
            return false;
        }
    }

    self->pio = pio;
    self->state_machine = state_machine;
    self->offset = offset;
    _current_program_id[pio_index][state_machine] = program_id;
    _current_program_len[pio_index][state_machine] = program_len;
    _current_program_offset[pio_index][state_machine] = offset;
    _current_sm_pins[pio_index][state_machine] = pins_we_use;
    PIO_PINMASK_MERGE(_current_pins[pio_index], pins_we_use);

    pio_sm_set_pins_with_mask64(self->pio, state_machine, PIO_PINMASK_VALUE(initial_pin_state), PIO_PINMASK_VALUE(pins_we_use));
    pio_sm_set_pindirs_with_mask64(self->pio, state_machine, PIO_PINMASK_VALUE(initial_pin_direction), PIO_PINMASK_VALUE(pins_we_use));
    rp2pio_statemachine_set_pull(pull_pin_up, pull_pin_down, pins_we_use);
    self->initial_pin_state = initial_pin_state;
    self->initial_pin_direction = initial_pin_direction;
    self->pull_pin_up = pull_pin_up;
    self->pull_pin_down = pull_pin_down;

    for (size_t pin_number = 0; pin_number < NUM_BANK0_GPIOS; pin_number++) {
        if (!PIO_PINMASK_IS_SET(pins_we_use, pin_number)) {
            continue;
        }
        const mcu_pin_obj_t *pin = mcu_get_pin_by_number(pin_number);
        if (!pin) {
            // TODO: should be impossible, but free resources here anyway
            return false;
        }
        _pin_reference_count[pin_number]++;
        // Also claim the pin at the top level when we're the first to grab it.
        if (_pin_reference_count[pin_number] == 1) {
            if (claim_pins) {
                claim_pin(pin);
            }
            pio_gpio_init(self->pio, pin_number);
        }

        // Use lowest drive level for all State Machine outputs. (#7515
        // workaround). Remove if/when Pin objects get a drive_strength
        // property and use that value instead.
        gpio_set_drive_strength(pin_number, GPIO_DRIVE_STRENGTH_2MA);
    }

    pio_sm_config c = pio_get_default_sm_config();

    if (frequency == 0) {
        frequency = clock_get_hz(clk_sys);
    }
    uint64_t frequency256 = ((uint64_t)clock_get_hz(clk_sys)) * 256;
    uint64_t div256 = frequency256 / frequency;
    if (frequency256 % div256 > 0) {
        div256 += 1;
    }
    self->actual_frequency = frequency256 / div256;
    sm_config_set_clkdiv_int_frac8(&c, div256 / 256, div256 % 256);

    if (first_out_pin != NULL) {
        sm_config_set_out_pins(&c, first_out_pin->number, out_pin_count);
    }
    if (first_in_pin != NULL) {
        sm_config_set_in_pins(&c, first_in_pin->number);
    }
    if (first_set_pin != NULL) {
        sm_config_set_set_pins(&c, first_set_pin->number, set_pin_count);
    }
    if (first_sideset_pin != NULL) {
        size_t total_sideset_bits = sideset_pin_count;
        if (sideset_enable) {
            total_sideset_bits += 1;
        }
        sm_config_set_sideset(&c, total_sideset_bits, sideset_enable, sideset_pindirs);
        sm_config_set_sideset_pins(&c, first_sideset_pin->number);
    }
    if (jmp_pin != NULL) {
        sm_config_set_jmp_pin(&c, jmp_pin->number);
    }

    mp_arg_validate_int_range(wrap, -1, program_len - 1);
    if (wrap == -1) {
        wrap = program_len - 1;
    }

    mp_arg_validate_int_range(wrap_target, 0, program_len - 1);

    wrap += offset;
    wrap_target += offset;

    sm_config_set_wrap(&c, wrap_target, wrap);
    sm_config_set_in_shift(&c, in_shift_right, auto_push, push_threshold);
    #if PICO_PIO_VERSION > 0
    sm_config_set_in_pin_count(&c, in_pin_count);
    #endif

    sm_config_set_out_shift(&c, out_shift_right, auto_pull, pull_threshold);
    sm_config_set_out_pin_count(&c, out_pin_count);

    sm_config_set_set_pin_count(&c, set_pin_count);

    enum pio_fifo_join join = compute_fifo_type(fifo_type, rx_fifo, tx_fifo);

    self->fifo_depth = compute_fifo_depth(join);

    #if PICO_PIO_VERSION > 0
    if (fifo_type == PIO_FIFO_JOIN_TXPUT || fifo_type == PIO_FIFO_JOIN_TXGET) {
        printf("Non-supported\n");
        //self->rxfifo_obj.base.type = &memorymap_addressrange_type;
        //common_hal_memorymap_addressrange_construct(&self->rxfifo_obj, (uint8_t *)self->pio->rxf_putget[self->state_machine], 4 * sizeof(uint32_t));
    } else {
        self->rxfifo_obj.base.type = NULL;
    }
    #endif

    if (rx_fifo) {
        self->rx_dreq = pio_get_dreq(self->pio, self->state_machine, false);
    }
    if (tx_fifo) {
        self->tx_dreq = pio_get_dreq(self->pio, self->state_machine, true);
    }
    self->in = rx_fifo;
    self->out = tx_fifo;
    self->out_shift_right = out_shift_right;
    self->in_shift_right = in_shift_right;
    self->wait_for_txstall = wait_for_txstall;
    self->user_interruptible = user_interruptible;

    self->init = init;
    self->init_len = init_len;

    sm_config_set_fifo_join(&c, join);

    // TODO: these arguments
    // int mov_status_type, int mov_status_n,
    // int set_count, int out_count

    self->sm_config = c;

    // no DMA allocated
    SM_DMA_CLEAR_CHANNEL_READ(pio_index, state_machine);
    SM_DMA_CLEAR_CHANNEL_WRITE(pio_index, state_machine);

    pio_sm_init(self->pio, self->state_machine, offset, &c);
    common_hal_rp2pio_statemachine_run(self, init, init_len);

    common_hal_rp2pio_statemachine_set_frequency(self, frequency, frequency256, div256);
    pio_sm_set_enabled(self->pio, self->state_machine, true);
    return true;
}