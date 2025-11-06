#ifndef USB_MISC_H
#define USB_MISC_H

#include <stdint.h>

#define USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(ptr) \
    ((uint16_t)((uint16_t)(ptr)[0] | ((uint16_t)(ptr)[1] << 8)))

#define USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(ptr)                                                   \
    ((uint32_t)(ptr)[0] | ((uint32_t)(ptr)[1] << 8) | ((uint32_t)(ptr)[2] << 16) | ((uint32_t)(ptr)[3] << 24))

#endif /* USB_MISC_H */
