/*
 * Copyright (c) 2026 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Reference a weak function and a weak object that nothing defines. The
 * extension must still load, with both addresses resolving to NULL.
 */

#include <zephyr/llext/symbol.h>
#include <zephyr/ztest_assert.h>

extern void llext_weak_undef_fn(void);
extern int llext_weak_undef_obj;
#pragma weak llext_weak_undef_fn
#pragma weak llext_weak_undef_obj

void test_entry(void)
{
	zassert_is_null(llext_weak_undef_fn, "undefined weak function resolved to %p",
			llext_weak_undef_fn);
	zassert_is_null(&llext_weak_undef_obj, "undefined weak object resolved to %p",
			&llext_weak_undef_obj);
}
EXPORT_SYMBOL(test_entry);
