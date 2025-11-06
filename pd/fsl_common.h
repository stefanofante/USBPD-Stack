#ifndef FSL_COMMON_H
#define FSL_COMMON_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_WEAK_FUN __attribute__((weak))
#define USB_GLOBAL

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define SDK_ALIGN(x) __attribute__((aligned(x)))

#define FSL_RTOS_FREE_RTOS (1)

typedef int32_t status_t;

#ifdef __cplusplus
}
#endif

#endif /* FSL_COMMON_H */
