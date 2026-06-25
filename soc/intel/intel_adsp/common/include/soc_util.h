/*
 * Copyright (c) 2019-2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_SOC_INTEL_ADSP_COMMON_UTIL_H_
#define ZEPHYR_SOC_INTEL_ADSP_COMMON_UTIL_H_

#include <zephyr/cache.h>

/* memcopy used by boot loader */
static ALWAYS_INLINE void bmemcpy(void *dest, void *src, size_t bytes)
{
	uint8_t *d = (uint8_t *)dest;
	uint8_t *s = (uint8_t *)src;

	sys_cache_data_invd_range(src, bytes);
	for (size_t i = 0; i < bytes; i++) {
		d[i] = s[i];
	}

	sys_cache_data_flush_range(dest, bytes);
}

static ALWAYS_INLINE void bbzero(void *dest, size_t bytes)
{
	uint8_t *d = (uint8_t *)dest;

	for (size_t i = 0; i < bytes; i++) {
		d[i] = 0;
	}

	sys_cache_data_flush_range(dest, bytes);
}

#endif /* ZEPHYR_SOC_INTEL_ADSP_COMMON_UTIL_H_ */
