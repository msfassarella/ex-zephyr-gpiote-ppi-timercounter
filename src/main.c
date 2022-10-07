/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <nrfx_gpiote.h>
#include <nrfx_timer.h>

#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(nrfx_sample, LOG_LEVEL_INF);

#define INPUT_PIN DT_GPIO_PIN(DT_ALIAS(sw0), gpios)
#define INPUT_PIN1 DT_GPIO_PIN(DT_ALIAS(sw1), gpios)
#define OUTPUT_PIN DT_GPIO_PIN(DT_ALIAS(led0), gpios)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0 ""
#define PIN 0
#define FLAGS 0
#endif

const nrfx_timer_t timer0 = NRFX_TIMER_INSTANCE(0);

static void timer0_init(void)
{
	nrfx_timer_config_t timer0_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer0_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer0_config.frequency = NRF_TIMER_FREQ_16MHz;
	timer0_config.mode = NRF_TIMER_MODE_COUNTER;
	nrfx_timer_init(&timer0, &timer0_config, NULL);

	/*
	// If you wish clear the counter automaticaly, you can use the function below
	nrfx_timer_extended_compare(
         &timer0, NRF_TIMER_CC_CHANNEL0, 8, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	*/
}

static void button_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint32_t timer_value = 0;
	LOG_INF("GPIO input event callback. Source pin num %d", pin);
	switch (pin) {
	case INPUT_PIN: {
		nrfx_timer_clear(&timer0);
		LOG_INF("Clear CONT");
	} break;
	case INPUT_PIN1: {
		//nrfx_timer_clear(&timer0);
	} break;
	default:
		break;
	}

	timer_value = nrfx_timer_capture(&timer0, NRF_TIMER_CC_CHANNEL1);

	LOG_INF("CONT %d", timer_value);
}

void main(void)
{
	LOG_INF("nrfx_gpiote sample on %s", CONFIG_BOARD);

	nrfx_err_t err;

	/* Connect GPIOTE_0 IRQ to nrfx_gpiote_irq_handler */
	// To work without IRQ erro is necessary that IRQ_CONNECT and nrfx_gpiote_init be disable
	//IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpiote)), DT_IRQ(DT_NODELABEL(gpiote), priority), nrfx_isr,
	//	    nrfx_gpiote_irq_handler, 0);

	/* Initialize GPIOTE (the interrupt priority passed as the parameter
	 * here is ignored, see nrfx_glue.h).
	 */
	/*
	err = nrfx_gpiote_init(0);
    if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_init error: %08x", err);
		return;
	}
   */
	nrfx_gpiote_in_config_t const in_config = {
		.sense = NRF_GPIOTE_POLARITY_HITOLO,
		.pull = NRF_GPIO_PIN_PULLUP,
		.is_watcher = false,
		.hi_accuracy = true,
		.skip_gpio_setup = false,
	};

	/* Initialize input pin to generate event on high to low transition
	 * (falling edge) and call button_handler()
	 */
	err = nrfx_gpiote_in_init(INPUT_PIN, &in_config, button_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_in_init error: %08x", err);
		return;
	}

	err = nrfx_gpiote_in_init(INPUT_PIN1, &in_config, button_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("input1 nrfx_gpiote_in_init error: %08x", err);
		return;
	}

	nrfx_gpiote_out_config_t const out_config = {
		.action = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_state = 1,
		.task_pin = true,
	};

	/* Initialize output pin. SET task will turn the LED on,
	 * CLR will turn it off and OUT will toggle it.
	 */
	err = nrfx_gpiote_out_init(OUTPUT_PIN, &out_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_out_init error: %08x", err);
		return;
	}

	nrfx_gpiote_in_event_enable(INPUT_PIN, true);  //once per time
	nrfx_gpiote_in_event_enable(INPUT_PIN1, true); //once per time
	/* if nrfx_gpiote_out_task_enable is enable it generates an conflict with gpio_pin_toggle*/
	// nrfx_gpiote_out_task_enable(OUTPUT_PIN);

	LOG_INF("nrfx_gpiote initialized");

	/* Allocate a (D)PPI channel. */
#if defined(DPPI_PRESENT)
	uint8_t channel;
	err = nrfx_dppi_channel_alloc(&channel);
#else
	nrf_ppi_channel_t channel;
	err = nrfx_ppi_channel_alloc(&channel);
#endif
	if (err != NRFX_SUCCESS) {
		LOG_ERR("(D)PPI channel allocation error: %08x", err);
		return;
	}

	/* Configure endpoints of the channel so that the input pin event is
	 * connected with the output pin OUT task. This means that each time
	 * the button is pressed, the LED pin will be toggled.
	 */
	nrfx_gppi_channel_endpoints_setup(channel, nrfx_gpiote_in_event_addr_get(INPUT_PIN),
					  nrfx_gpiote_out_task_addr_get(OUTPUT_PIN));
	

	uint32_t gpiote_task_addr;
	gpiote_task_addr = nrfx_gpiote_in_event_addr_get(INPUT_PIN1);
	nrfx_ppi_channel_assign(channel, gpiote_task_addr,
				nrfx_timer_task_address_get(&timer0, NRF_TIMER_TASK_COUNT));

	/* Enable (D)PPI channel. */
#if defined(DPPI_PRESENT)
	err = nrfx_dppi_channel_enable(channel);
#else
	err = nrfx_ppi_channel_enable(channel);
#endif
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to enable (D)PPI channel, error: %08x", err);
		return;
	}

	LOG_INF("(D)PPI configured, leaving main()");

	timer0_init();
	nrfx_timer_enable(&timer0);

	uint32_t timer_value;
	const struct device *dev;
	dev = device_get_binding(LED0);
	if (dev == NULL) {
		LOG_INF("Erro 1");
		return;
	}

	int ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		LOG_INF("Erro 2");
		return;
	}
	while (1) {
		timer_value = nrfx_timer_capture(&timer0, NRF_TIMER_CC_CHANNEL1);

		gpio_pin_toggle(dev, OUTPUT_PIN);
		LOG_INF("CONT %d", timer_value);
		k_msleep(1000);
	}
}
