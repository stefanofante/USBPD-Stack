#ifndef USB_PD_CONFIG_H
#define USB_PD_CONFIG_H

#include <stdint.h>

/* Target platform selector */
#define PD_CONFIG_TARGET_ESP32S3 (1)

/* Core stack configuration */
#define PD_CONFIG_MAX_PORT                          (1)
#define PD_CONFIG_REVISION                          (0x02U)
#define PD_CONFIG_STRUCTURED_VDM_VERSION            (0x01U)
#define PD_CONFIG_COMMON_TASK                       (0)
#define PD_CONFIG_ALT_MODE_SUPPORT                  (0)
#define PD_CONFIG_ENABLE_AUTO_POLICY                (0)
#define PD_CONFIG_ENABLE_AUTO_POLICY_LOG            (0)
#define PD_CONFIG_EXTENDED_MSG_SUPPORT              (1)
#define PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT      (1)
#define PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE     (0)
#define PD_CONFIG_PD3_PPS_ENABLE                    (0)
#define PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE         (0)
#define PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE    (0)
#define PD_CONFIG_COMPLIANCE_TEST_ENABLE            (0)
#define PD_CONFIG_EXTERNAL_POWER_DETECTION_SUPPORT  (0)
#define PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE         (0)
#define PD_CONFIG_CABLE_COMMUNICATION_ENABLE        (0)
#define PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG      (0)
#define PD_CONFIG_AUDIO_ACCESSORY_SUPPORT           (0)
#define PD_CONFIG_DEBUG_ACCESSORY_SUPPORT           (0)
#define PD_CONFIG_SINK_ACCESSORY_SUPPORT            (0)

/* Role and capability configuration */
#define PD_CONFIG_SOURCE_ROLE_ENABLE                (1)
#define PD_CONFIG_SINK_ROLE_ENABLE                  (1)
#define PD_CONFIG_DUAL_POWER_ROLE_ENABLE            (1)
#define PD_CONFIG_DUAL_DATA_ROLE_ENABLE             (1)
#define PD_CONFIG_TRY_SRC_SUPPORT                   (1)
#define PD_CONFIG_TRY_SNK_SUPPORT                   (0)
#define PD_CONFIG_VCONN_SUPPORT                     (1)
#define PD_CONFIG_PHY_LOW_POWER_LEVEL               (0)
#define PD_CONFIG_VBUS_ALARM_SUPPORT                (0)

/* TCPC / PHY selection */
#define PD_CONFIG_PTN5110_PORT                      (1)
#define PD_CONFIG_PTN5100_PORT                      (0)

/* Debug accessory role (not used on ESP32-S3) */
#define PD_CONFIG_DEBUG_ACCESSORY_ROLE              (CONFIG_DEBUG_ACCESSORY_NONE)

/* Sink detach detection method */
#define PD_CONFIG_SINK_DETACH_DETECT_WAY            (PD_SINK_DETACH_ON_VBUS_ABSENT)

/* Alternate mode feature flags (disabled) */
#define PD_CONFIG_ALT_MODE_DP_SUPPORT               (0)
#define PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE      (0)
#define PD_CONFIG_ALT_MODE_HOST_SUPPORT             (0)
#define PD_CONFIG_ALT_MODE_SLAVE_SUPPORT            (0)

#endif /* USB_PD_CONFIG_H */
