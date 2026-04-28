/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_gpio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/clock/lpc84x-clock.h>
#include <zephyr/logging/log.h>

#include <fsl_gpio.h>
#include <fsl_iocon.h>
#include <soc.h>

LOG_MODULE_REGISTER(gpio_lpc84x, CONFIG_GPIO_LOG_LEVEL);

/**
 * Hardware limitation check:
 * PINT channels 6 and 7 share NVIC vectors (IRQ 30 and 31) with UART3 and UART4.
 * If uart3 or uart4 are enabled alongside GPIO interrupts on PINT
 * channels 6 or 7, a fault will occur.
 */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(pint))

BUILD_ASSERT(!(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart3))),
	     "Hardware conflict: PINT (channel 6) and UART3 share IRQ 30."
	     "Both cannot be enabled simultaneously.");

BUILD_ASSERT(!(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart4))),
	     "Hardware conflict: PINT (channel 7) and UART4 share IRQ 31."
	     "Both cannot be enabled simultaneously.");

#endif

#define LPC84X_MAX_GPIO_PORTS 2

struct gpio_lpc84x_shared {
	SYSCON_Type *syscon;
	PINT_Type *pint;
	uint8_t nirqs;
	uint8_t pint_used_mask;
	/* Array to store initialized GPIO port devices */
	const struct device *port_devs[LPC84X_MAX_GPIO_PORTS];
};

struct gpio_lpc84x_config {
	DEVICE_MMIO_NAMED_ROM(regs);
	DEVICE_MMIO_NAMED_ROM(syscon);
	DEVICE_MMIO_NAMED_ROM(pint);
	struct gpio_driver_config common;
	const struct device *clock_dev;
	uint8_t port_no;
	uint8_t ngpios;
	clock_control_subsys_t clock_subsys;
	clock_control_subsys_t pint_clock;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_lpc84x_shared *shared;
};

struct gpio_lpc84x_data {
	DEVICE_MMIO_NAMED_RAM(regs);
	DEVICE_MMIO_NAMED_RAM(syscon);
	DEVICE_MMIO_NAMED_RAM(pint);
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

#define DEV_CFG(dev)  ((const struct gpio_lpc84x_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_lpc84x_data *)(dev)->data)

static int gpio_lpc84x_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_lpc84x_config *config = dev->config;
	struct gpio_lpc84x_data *data = dev->data;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);
	uint8_t idx;
	uint32_t pinconfig = 0;

	if ((config->port_no == 0) && (pin > config->ngpios)) {
		pin = (pin & 0x1f);
	}

	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	/*
	 * PIO0_10 and PIO0_11 are true open drain pins muxed with I2C port 0.
	 * Can be configured as GPIO's but only in open drain mode and with
	 * out pull-down or pull-up resisters enabled.
	 */
	if (config->port_no == 0 && (pin == 10 || pin == 11) &&
	    ((flags & GPIO_OPEN_DRAIN) == 0 || (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)))) {
		return -EINVAL;
	}

	if (flags & GPIO_PULL_UP) {
		pinconfig |= IOCON_PIO_MODE(0x2);
	} else if (flags & GPIO_PULL_DOWN) {
		pinconfig |= IOCON_PIO_MODE(0x1);
	} else {
		pinconfig |= IOCON_PIO_MODE(0x0);
	}

	if (flags & GPIO_OPEN_DRAIN) {
		pinconfig |= IOCON_PIO_OD(0x1);
	}

	/* Set the pin value before setting it as an output to avoid glitches. */
	if (flags & GPIO_OUTPUT_INIT_LOW) {
		GPIO_PinWrite(base, config->port_no, pin, 0);
	} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
		GPIO_PinWrite(base, config->port_no, pin, 1);
	}

	/* Configure GPIO direction. */
	if (flags & GPIO_OUTPUT) {
		base->DIRSET[config->port_no] = BIT(pin);
	} else {
		/* GPIO_INPUT and GPIO_DISCONNECTED (= 0): both become high-Z input. */
		base->DIRCLR[config->port_no] = BIT(pin);
	}

	idx = lpc84x_iocon_index(config->port_no, pin);

	pinctrl_soc_pin_t pin_cfg = {.swm_cfg = LPC84X_SWM_NONE,
				     .iocon_cfg = idx | ((pinconfig >> 3) << 8)};

	pinctrl_configure_pins(&pin_cfg, 1, 0);

	if (flags & GPIO_ACTIVE_LOW) {
		data->common.invert |= (gpio_port_pins_t)BIT(pin);
	} else {
		data->common.invert &= ~(gpio_port_pins_t)BIT(pin);
	}

	return 0;
}

static int gpio_lpc84x_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	const struct gpio_lpc84x_config *config = dev->config;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);

	GPIO_PortSet(base, config->port_no, (mask & value));

	GPIO_PortClear(base, config->port_no, (mask & ~value));

	return 0;
}

static int gpio_lpc84x_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_lpc84x_config *config = dev->config;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);

	GPIO_PortClear(base, config->port_no, mask);

	return 0;
}

static int gpio_lpc84x_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_lpc84x_config *config = dev->config;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);

	*value = GPIO_PortRead(base, config->port_no);

	return 0;
}

static int gpio_lpc84x_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_lpc84x_config *config = dev->config;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);

	GPIO_PortSet(base, config->port_no, mask);

	return 0;
}

static int gpio_lpc84x_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_lpc84x_config *config = dev->config;
	GPIO_Type *base = (GPIO_Type *)DEVICE_MMIO_NAMED_GET(dev, regs);

	GPIO_PortToggle(base, config->port_no, mask);

	return 0;
}

/**
 * @brief Allocates a free Pin Interrupt (PINT) slot and configures the
 * SYSCON multiplexer to route the specified GPIO pin to it.
 *
 * @param shared   Pointer to the shared state structure for all GPIO ports.
 * @param intpin   The encoded GPIO port and pin number to attach.
 *
 * @retval >= 0    The index of the successfully attached PINT slot.
 * @retval -EBUSY  All PINT interrupt lines are currently in use.
 */
static int pin_attach(struct gpio_lpc84x_shared *shared, uint8_t intpin)
{
	uint8_t irq;

	for (irq = 0; irq < shared->nirqs; irq++) {
		if ((shared->syscon->PINTSEL[irq] & SYSCON_PINTSEL_INTPIN_MASK) == intpin &&
		    (shared->pint_used_mask & BIT(irq))) {
			return irq;
		}
	}

	for (irq = 0; irq < shared->nirqs; irq++) {
		if ((shared->pint_used_mask & BIT(irq)) == 0) {
			shared->syscon->PINTSEL[irq] = intpin;
			shared->pint_used_mask |= BIT(irq);
			return irq;
		}
	}

	return -EBUSY;
}

/**
 * @brief Frees the assigned Pin Interrupt (PINT) slot for a GPIO pin and
 * clears its routing in the SYSCON multiplexer.
 *
 * @param shared   Pointer to the shared state structure for all GPIO ports.
 * @param intpin   The encoded GPIO port and pin number to detach.
 *
 * @retval >= 0    The index of the successfully detached PINT slot.
 * @retval -EINVAL No active attached interrupt was found for the requested GPIO.
 */
static int pin_detach(struct gpio_lpc84x_shared *shared, uint8_t intpin)
{
	uint8_t irq;

	for (irq = 0; irq < shared->nirqs; irq++) {
		if ((shared->syscon->PINTSEL[irq] & SYSCON_PINTSEL_INTPIN_MASK) == intpin &&
		    shared->pint_used_mask & BIT(irq)) {
			shared->pint_used_mask &= ~BIT(irq);
			/* Clear the stale entry */
			shared->syscon->PINTSEL[irq] = 0;
			return irq;
		}
	}

	return -EINVAL;
}

static int gpio_lpc84x_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_lpc84x_config *config = dev->config;
	struct gpio_lpc84x_shared *shared = config->shared;
	uint8_t intpin = pin + (config->port_no * 32);
	uint32_t irq = 0;

	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		irq = pin_detach(shared, intpin);
		if (irq >= 0) {
			/* Disable interrupts and clear pending flags. */
			shared->pint->CIENR = BIT(irq);
			shared->pint->CIENF = BIT(irq);
			shared->pint->ISEL &= ~BIT(irq);
			shared->pint->IST = BIT(irq);
		}
		return 0;
	}

	irq = pin_attach(shared, intpin);
	if (irq < 0) {
		return irq;
	}

	/* Disable and clear any stale latch before reconfiguring. */
	shared->pint->CIENR = BIT(irq);
	shared->pint->CIENF = BIT(irq);
	shared->pint->ISEL &= ~BIT(irq);
	shared->pint->IST = BIT(irq);

	switch (mode) {
	case GPIO_INT_MODE_EDGE:
		/* Edge interrupt mode. */
		shared->pint->ISEL &= ~BIT(irq);
		/* Enable interrupts on falling and/or rising edges. */
		if (trig & GPIO_INT_TRIG_LOW) {
			shared->pint->SIENF = BIT(irq);
		} else {
			shared->pint->CIENF = BIT(irq);
		}

		if (trig & GPIO_INT_TRIG_HIGH) {
			shared->pint->SIENR = BIT(irq);
		} else {
			shared->pint->CIENR = BIT(irq);
		}
		break;
	case GPIO_INT_MODE_LEVEL:
		/* Level interrupt mode. */
		shared->pint->ISEL |= BIT(irq);
		/* Level-sensitive interrupts, IENR enables the interrupt. */
		shared->pint->SIENR = BIT(irq);
		if (trig & GPIO_INT_TRIG_LOW) {
			/* Active LOW interrupt selected. */
			shared->pint->SIENF = BIT(irq);
		} else {
			/* Active HIGH interrupt selected. */
			shared->pint->CIENF = BIT(irq);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_lpc84x_manage_callback(const struct device *dev, struct gpio_callback *call_back,
				       bool set)
{
	struct gpio_lpc84x_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, call_back, set);
}

static void gpio_lpc84x_isr(const void *arg)
{
	struct gpio_lpc84x_shared *shared = (struct gpio_lpc84x_shared *)arg;
	uint32_t active_pins[LPC84X_MAX_GPIO_PORTS] = {0, 0};

	/* LOOP 1: Check hardware interrupts and build the active_pins masks */
	for (uint8_t irq = 0; irq < shared->nirqs; irq++) {
		if (shared->pint->IST & BIT(irq)) {
			/* Clear interrupt status. */
			if (!(shared->pint->ISEL & BIT(irq))) {
				shared->pint->IST = BIT(irq);
			}

			/* intpin extract GPIO port and pin number. */
			uint8_t intpin = shared->syscon->PINTSEL[irq] & SYSCON_PINTSEL_INTPIN_MASK;

			if (intpin < 32) {
				active_pins[0] |= BIT(intpin);
			} else if (intpin < 54) {
				active_pins[1] |= BIT(intpin - 32);
			}
		}
	}

	/* LOOP 2: Trigger Zephyr callbacks for any active, initialized ports */
	for (uint8_t i = 0; i < LPC84X_MAX_GPIO_PORTS; i++) {
		if (active_pins[i] != 0 && shared->port_devs[i] != NULL) {
			struct gpio_lpc84x_data *data = shared->port_devs[i]->data;

			gpio_fire_callbacks(&data->callbacks, shared->port_devs[i], active_pins[i]);
		}
	}
}

static struct gpio_lpc84x_shared gpio_lpc84x_shared = {
	.pint_used_mask = 0,
};

#define PINT_NODE DT_NODELABEL(pint)

#define PINT_IRQ_CONNECT(n)                                                                        \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_IDX(PINT_NODE, n, irq),                                      \
			    DT_IRQ_BY_IDX(PINT_NODE, n, priority), gpio_lpc84x_isr,                \
			    &gpio_lpc84x_shared, 0);                                               \
		irq_enable(DT_IRQ_BY_IDX(PINT_NODE, n, irq));                                      \
	} while (false)

static int gpio_lpc84x_init(const struct device *dev)
{
	const struct gpio_lpc84x_config *config = dev->config;
	struct gpio_lpc84x_shared *shared = config->shared;

	int err;
	static atomic_t initialized = ATOMIC_INIT(0);

	DEVICE_MMIO_NAMED_MAP(dev, regs, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, syscon, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, pint, K_MEM_CACHE_NONE);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err && err != -ENOENT) {
		LOG_ERR("Failed to apply pinctrl state (err %d)", err);
		return err;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err < 0) {
		LOG_ERR("Failed to enable clock (err %d)", err);
		return err;
	}

	if (!atomic_test_and_set_bit(&initialized, 0)) {
		shared->syscon = (SYSCON_Type *)DEVICE_MMIO_NAMED_GET(dev, syscon);
		shared->pint = (PINT_Type *)DEVICE_MMIO_NAMED_GET(dev, pint);
		shared->nirqs = DT_PROP(DT_NODELABEL(pint), num_lines);

		err = clock_control_on(config->clock_dev, config->pint_clock);
		if (err < 0) {
			LOG_ERR("failed to enable the gpio interrupt clock (err %d)", err);
			return err;
		}

		PINT_IRQ_CONNECT(0);
		PINT_IRQ_CONNECT(1);
		PINT_IRQ_CONNECT(2);
		PINT_IRQ_CONNECT(3);
		PINT_IRQ_CONNECT(4);
		PINT_IRQ_CONNECT(5);
		PINT_IRQ_CONNECT(6);
		PINT_IRQ_CONNECT(7);
	}

	if (config->port_no < LPC84X_MAX_GPIO_PORTS) {
		shared->port_devs[config->port_no] = dev;
	}

	return 0;
}

static DEVICE_API(gpio, gpio_lpc84x_driver_api) = {
	.pin_configure = gpio_lpc84x_pin_configure,
	.port_get_raw = gpio_lpc84x_port_get_raw,
	.port_set_masked_raw = gpio_lpc84x_port_set_masked_raw,
	.port_set_bits_raw = gpio_lpc84x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_lpc84x_port_clear_bits_raw,
	.port_toggle_bits = gpio_lpc84x_port_toggle_bits,
	.pin_interrupt_configure = gpio_lpc84x_pin_interrupt_configure,
	.manage_callback = gpio_lpc84x_manage_callback,
};

#define GPIO_LPC84X_INIT(id)                                                                       \
	PINCTRL_DT_INST_DEFINE(id);                                                                \
                                                                                                   \
	static const struct gpio_lpc84x_config gpio_lpc84x_config_##id = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(id),              \
			},                                                                         \
		DEVICE_MMIO_NAMED_ROM_INIT(regs, DT_INST_PARENT(id)),                              \
		DEVICE_MMIO_NAMED_ROM_INIT(syscon, DT_NODELABEL(syscon)),                          \
		DEVICE_MMIO_NAMED_ROM_INIT(pint, DT_NODELABEL(pint)),                              \
		.port_no = DT_INST_PROP(id, port_no),                                              \
		.ngpios = DT_INST_PROP(id, ngpios),                                                \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(id, clocks)),                           \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(id, name),             \
		.pint_clock = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_NODELABEL(pint), name),    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                                        \
		.shared = &gpio_lpc84x_shared,                                                     \
	};                                                                                         \
                                                                                                   \
	static struct gpio_lpc84x_data gpio_lpc84x_data_##id;                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, gpio_lpc84x_init, NULL, &gpio_lpc84x_data_##id,                  \
			      &gpio_lpc84x_config_##id, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,   \
			      &gpio_lpc84x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_LPC84X_INIT)
