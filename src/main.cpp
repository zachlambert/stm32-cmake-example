#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/cm3/itm.h>

#include <cstring>
#include <nanoprintf.h>
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
    DBGMCU_CR = DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_ASYNC;

    *((volatile uint32_t *)0xE0000FB0) = 0xC5ACCE55; // **ACCESS

    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    ITM_TCR = (1 << 16) | ITM_TCR_ITMENA;
    ITM_TER[0] |= (1 << itm_stimulus_port);
    ITM_TPR |= 0x01;
    ITM_STIM8(itm_stimulus_port) = 0x00;
}

static void trace_send_blocking(const char* string, size_t length) {
    for (size_t i = 0; i < length; i++) {
        while (!(ITM_STIM8(itm_stimulus_port) & ITM_STIM_FIFOREADY));
        ITM_STIM8(itm_stimulus_port) = string[i];
    }
}

void itm_printf(char const *format, ...) {
    char buffer[64];
    va_list values;
    va_start(values, format);
    int const length = npf_vsnprintf(buffer, sizeof(buffer), format, values);
    va_end(values);
    trace_send_blocking(buffer, length);
}

int main() {
    clock_setup();
    gpio_setup();
    delay_setup();
    trace_setup();

    preinit_usb();
    delay_ms(3);
    auto usbd_dev = init_usb();
    if (usbd_dev) {
        gpio_set(GPIOA, GPIO5);
    }

    int counter = 0;
    while (true) {
        gpio_toggle(GPIOA, GPIO4);

        if (usbd_dev) {
            char message[128];
            size_t message_len = npf_snprintf(message, sizeof(message), "usb: %i\n", counter);
            usbd_ep_write_packet(usbd_dev, 0x82, message, message_len);
        }

        itm_printf("swv: %i\n", counter);

        counter++;
        delay_ms(100);
    }
}
