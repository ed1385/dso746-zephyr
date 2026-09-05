/* host stand-in for the Zephyr kernel: enough for ui.c / sim.c */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
struct k_mutex { int depth; };
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_FOREVER 0
static inline int k_mutex_lock(struct k_mutex *m, int t) { (void)t; m->depth++; return 0; }
static inline int k_mutex_unlock(struct k_mutex *m) { m->depth--; return 0; }
extern uint32_t sim_now_ms;
static inline uint32_t k_uptime_get_32(void) { return sim_now_ms; }
#define ARG_UNUSED(x) (void)(x)
#define __aligned(x) __attribute__((aligned(x)))
#define BUILD_ASSERT(c, ...) _Static_assert(c, "build assert")
