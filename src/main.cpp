
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/cm3/itm.h>

#include "usb.h"

static const int itm_stimulus_port = 0;

static void clock_setup(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE16_72MHZ]);
}

static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO4 | GPIO5
    );
}

static void delay_setup(void) {
	rcc_periph_clock_enable(RCC_TIM2);
	timer_set_prescaler(TIM2, rcc_apb1_frequency / 500000 - 1);
	timer_one_shot_mode(TIM2);
}

static void delay_us(uint16_t us) {
    timer_set_period(TIM2, us);
    timer_enable_update_event(TIM2);
	timer_enable_counter(TIM2);
	while (TIM_CR1(TIM2) & TIM_CR1_CEN);
}

static void delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

static void trace_setup(void) {
#if 1
    DBGMCU_CR = DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_ASYNC;

    *((volatile uint32_t *)0xE0000FB0) = 0xC5ACCE55;
    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    ITM_TCR = (1 << 16) | ITM_TCR_ITMENA;
    ITM_TER[0] |= (1 << itm_stimulus_port);
    ITM_TPR |= 0x01;
    ITM_STIM8(itm_stimulus_port) = 0x00;

    // ITM_TCR |= ITM_TCR_TSPRESCALE_DIV16;
#endif

#if 0
    /* Enable trace subsystem (we'll use ITM and TPIU). */
    SCS_DEMCR |= SCS_DEMCR_TRCENA;

    /* Use Manchester code for asynchronous transmission. */
    TPIU_SPPR = TPIU_SPPR_ASYNC_MANCHESTER;
    TPIU_ACPR = 7;

    /* Formatter and flush control. */
    TPIU_FFCR &= ~TPIU_FFCR_ENFCONT;

    /* Enable TRACESWO pin for async mode. */
    DBGMCU_CR = DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_ASYNC;

    /* Unlock access to ITM registers. */
    /* FIXME: Magic numbers... Is this Cortex-M3 generic? */
    *((volatile uint32_t *)0xE0000FB0) = 0xC5ACCE55;

    /* Enable ITM with ID = itm_stimulus_port. */
    ITM_TCR = (1 << 16) | ITM_TCR_ITMENA;
    /* Enable stimulus port 1. */
    ITM_TER[0] = (1 << itm_stimulus_port);
#endif
}

static void trace_send_blocking(char c) {
    while (!(ITM_STIM8(itm_stimulus_port) & ITM_STIM_FIFOREADY));
    ITM_STIM8(itm_stimulus_port) = c;
}

int main() {
    clock_setup();
    gpio_setup();
    delay_setup();
    trace_setup();

    gpio_set(GPIOA, GPIO5);

    // auto usbd_dev = init_usb();

    int c = 0;
    int j = 0;

    while (true) {
        gpio_toggle(GPIOA, GPIO4);

        const char message[] = "hello\r\n";
        const size_t message_len = strnlen(message, sizeof(message));
        for (size_t i = 0; i < message_len; i++) {
            trace_send_blocking(message[i]);
        }

        // trace_send_blocking(c + '0');
        // c = (c == 9) ? 0 : c + 1;	/* Increment c. */
        // if ((j++ % 10) == 0) {		/* Newline after line full. */
        //     trace_send_blocking('\r');
        //     trace_send_blocking('\n');
        // }

        delay_ms(1);
    }
}
