/*
 * Copyright (c) 2024 Safae Ouajih <ouajih.safae@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_rng200

#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/sys_io.h>
#include <string.h>

#define BCM2711_RNG_RBGEN_MASK			0x00001FFF
#define BCM2711_RNG_FIFO_COUNT_MASK		0x000000FF
#define BCM2711_RNG_STATUS_INT_FAIL_MASK	0x80000020

#define BCM2711_RNG_STATUS_OFFSET		0x18
#define BCM2711_RNG_DATA_OFFSET			0x20
#define BCM2711_RNG_FIFO_COUNT_OFFSET		0x24

struct bcm2711_rng200_data {
	DEVICE_MMIO_RAM;
	mm_reg_t rng_addr;
};

struct bcm2711_rng200_config {
	DEVICE_MMIO_ROM;
};

static inline void bcm2711_rng200_enable(const struct device *dev)
{
	struct bcm2711_rng200_data *dev_data = dev->data;
	mm_reg_t base = dev_data->rng_addr;
	uint32_t value = sys_read32(base);

	value |= BCM2711_RNG_RBGEN_MASK;
	sys_write32(value, base);
}

static inline void bcm2711_rng200_disable(const struct device *dev)
{
	struct bcm2711_rng200_data *dev_data = dev->data;
	mm_reg_t base = dev_data->rng_addr;
	uint32_t value = sys_read32(base);

	value &= ~BCM2711_RNG_RBGEN_MASK;
	sys_write32(value, base);
}

static inline void bcm2711_rng200_get_word(const struct device *dev, uint8_t buf[4])
{
	struct bcm2711_rng200_data *dev_data = dev->data;
	mm_reg_t base = dev_data->rng_addr;
	uint32_t word = 0;

	word = sys_read32(base + BCM2711_RNG_DATA_OFFSET);

	buf[0] = (uint8_t)word;
	buf[1] = (uint8_t)(word >> 8);
	buf[2] = (uint8_t)(word >> 16);
	buf[3] = (uint8_t)(word >> 24);
}


static int entropy_bcm2711_rng200_get_entropy(const struct device *dev, uint8_t *buffer,
					      uint16_t length)
{
	ARG_UNUSED(dev);
	struct bcm2711_rng200_data *dev_data = dev->data;
	mm_reg_t base = dev_data->rng_addr;
	uint32_t count = 0;
	uint32_t status = 0;
	uint16_t remain = length;
	bool retry = true;
	uint8_t buf[4];

	while (remain) {
		status = sys_read32(base + BCM2711_RNG_STATUS_OFFSET);
		/* TODO restart when interrupt fail */
		if (status & BCM2711_RNG_STATUS_INT_FAIL_MASK)
			return length - remain;

		count = sys_read32(base + BCM2711_RNG_FIFO_COUNT_OFFSET);
		if (!(count & BCM2711_RNG_FIFO_COUNT_MASK)) {
			if (retry) {
				k_sleep(K_USEC(500));
				retry = false;
			} else
				return length - remain;
		} else {
			if (remain < sizeof(uint32_t)) {
				bcm2711_rng200_get_word(dev, buf);
				memcpy(buffer, buf, remain);
				buffer += remain;
				remain = 0;
			} else {
				bcm2711_rng200_get_word(dev, buffer);
				buffer += sizeof(uint32_t);
				remain -= sizeof(uint32_t);
			}
			retry = true;
		}
	}

	return remain ? -ENODEV : 0;
}

static int entropy_bcm2711_rng200_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	struct bcm2711_rng200_data *dev_data = dev->data;

	dev_data->rng_addr = DEVICE_MMIO_GET(dev);
	bcm2711_rng200_enable(dev);

	return 0;
}

static const struct entropy_driver_api entropy_bcm2711_rng200_api = {
	.get_entropy = entropy_bcm2711_rng200_get_entropy
};

#define BCM2711_RNG_INIT(idx) \
	static struct bcm2711_rng200_data bcm2711_rng200_##idx##_data; \
	static struct bcm2711_rng200_config bcm2711_rng200_##idx##_config = { \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(idx)), \
	}; \
	DEVICE_DT_INST_DEFINE(idx,\
			      entropy_bcm2711_rng200_init,\
			      NULL, &bcm2711_rng200_##idx##_data, &bcm2711_rng200_##idx##_config,\
			      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,\
			      &entropy_bcm2711_rng200_api);

DT_INST_FOREACH_STATUS_OKAY(BCM2711_RNG_INIT)
