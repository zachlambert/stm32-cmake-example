#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/cm3/itm.h>

// #include <cstdint>
// #include <micro_types/vector.hpp>
// #include <datapack/datapack.hpp>
// #include <datapack/common.hpp>
// #include <datapack/format/binary_writer.hpp>

#include <cstring>
#include "usb.h"
#include <nanoprintf.h>


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

#if 0
struct MyData {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
};
DATAPACK(MyData);
template <typename Visitor>
void visit(Visitor& visitor, MyData& value) {
    visitor.object_begin();
    visitor.value("x", value.x);
    visitor.value("y", value.y);
    visitor.value("z", value.z);
    visitor.object_end();
}
DATAPACK_IMPL(MyData);

mct::static_vector<std::uint8_t, 1024> binary_buffer;
#endif

int main() {
    clock_setup();
    gpio_setup();
    delay_setup();
    trace_setup();

#if 0
    MyData data;
    data.x = 10;
    data.y = 20;
    data.z = 30;
#endif

    preinit_usb();
    delay_ms(3);
    auto usbd_dev = init_usb();
    if (usbd_dev) {
        gpio_set(GPIOA, GPIO5);
    }

    int counter = 0;
    while (true) {
        gpio_toggle(GPIOA, GPIO4);

#if 0
        if (usbd_dev) {
            datapack::BinaryWriter writer(binary_buffer);
            writer.value(data);
            usbd_ep_write_packet(usbd_dev, 0x82, binary_buffer.data(), binary_buffer.size());
            itm_printf("Data size: %i\n", binary_buffer.size());
            for (std::size_t i = 0; i < binary_buffer.size(); i++) {
                itm_printf("%02x ", binary_buffer[i]);
                if ((i + 1) % 8 == 0) {
                    itm_printf("\n");
                }
            }
            if (binary_buffer.size() % 8 != 0) {
                itm_printf("\n");
            }
        }
#endif

#if 1
        if (usbd_dev) {
            char message[128] = "hello\n";
            size_t message_len = strnlen(message, sizeof(message));
            // size_t message_len = npf_snprintf(message, sizeof(message), "usb: %i\n", counter);
            usbd_ep_write_packet(usbd_dev, 0x82, message, message_len);
        }
#endif

        itm_printf("swv: %i\n", counter);

        counter++;
        delay_ms(100);
    }
}
