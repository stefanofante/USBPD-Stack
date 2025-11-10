/**
 * @file usb_pd_auto_policy.h
 * @brief USB PD Automatic Policy Configuration
 * 
 * @details Defines automatic policy negotiation settings for USB Power Delivery.
 *          Auto policy enables the stack to automatically:
 *          - Request power role swap (PR_Swap) when advantageous
 *          - Request data role swap (DR_Swap) to preferred role
 *          - Request VCONN swap to optimize power delivery
 *          - Accept/reject swap requests from partner based on configuration
 *          - Negotiate maximum available power as sink
 * 
 *          Configuration is bitfield-based for compact memory footprint.
 *          Each swap type has separate request and accept policies.
 *          Supports Accept, Reject, Wait, and NotSupport responses.
 * 
 *          Macro API provides type-safe access to configuration fields:
 *          - PD_POLICY_SUPPORT() - check if auto policy enabled
 *          - PD_POLICY_GET/SET_AUTO_REQUEST_* - control initiator behavior
 *          - PD_POLICY_GET/SET_AUTO_ACCEPT_* - control responder behavior
 * 
 * @copyright Copyright 2018 NXP. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_PD_AUTO_POLICY_H__
#define __USB_PD_AUTO_POLICY_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup usb_pd_stack
 * @{
 */

/**
 * @brief Auto policy accept/reject values
 * 
 * Response codes for automatic policy decisions on swap requests.
 */
typedef enum _usb_pd_auto_accept_value
{
    kAutoRequestProcess_NotSupport = 0x00u, /**< Feature not supported */
    kAutoRequestProcess_Accept = 0x01u,     /**< Auto accept request */
    kAutoRequestProcess_Reject = 0x02u,     /**< Auto reject request */
    kAutoRequestProcess_Wait = 0x03u,       /**< Auto reply wait for request */
} usb_pd_auto_accept_value_t;

/**
 * @brief Auto policy configuration structure
 * 
 * Bitfield configuration for automatic USB PD policy negotiation.
 * Controls both initiator (auto request) and responder (auto accept)
 * behavior for PR_Swap, DR_Swap, and VCONN_Swap operations.
 */
typedef struct _usb_pd_auto_policy
{
    /** Auto request PR_Swap when current power role is source (0=disabled, 1=enabled) */
    uint32_t autoRequestPRSwapAsSource : 1;
    
    /** Auto request PR_Swap when current power role is sink (0=disabled, 1=enabled) */
    uint32_t autoRequestPRSwapAsSink : 1;
    
    /** Accept PR_Swap when current role is source (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptPRSwapAsSource : 2;
    
    /** Accept PR_Swap when current role is sink (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptPRSwapAsSink : 2;
    
    /** Auto request DR_Swap to specified role (pd_data_role_t: kPD_DataRoleUFP/kPD_DataRoleDFP/kPD_DataRoleNone) */
    uint32_t autoRequestDRSwap : 2;
    
    /** Accept DR_Swap to DFP (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptDRSwapToDFP : 2;
    
    /** Accept DR_Swap to UFP (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptDRSwapToUFP : 2;
    
    /** Auto request VCONN_Swap (pd_vconn_role_t: kPD_NotVconnSource/kPD_IsVconnSource/kPD_VconnNone) */
    uint32_t autoRequestVConnSwap : 2;
    
    /** Accept VCONN_Swap to turn on VCONN (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptVconnSwapToOn : 2;
    
    /** Accept VCONN_Swap to turn off VCONN (usb_pd_auto_accept_value_t) */
    uint32_t autoAcceptVconnSwapToOff : 2;
    
    /** Sink auto-negotiate maximum power from source capabilities (0=disabled, 1=enabled) */
    uint32_t autoSinkNegotiation : 1;
    
    /** Reserved bits for future use */
    uint32_t reserved : 13;
    
    /** Reserved word for future expansion */
    uint32_t reserved1 : 32;
} pd_auto_policy_t;

/*! @}*/

/*******************************************************************************
 * API
 ******************************************************************************/

/** @brief Check if auto policy is configured */
#define PD_POLICY_SUPPORT(pdHandle) (((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig != NULL)

/* Power Role Swap Request Macros */

/** @brief Get auto request PR_Swap as source setting */
#define PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SOURCE(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestPRSwapAsSource

/** @brief Get auto request PR_Swap as sink setting */
#define PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SINK(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestPRSwapAsSink

/* Power Role Swap Accept Macros */

/** @brief Check if auto accept PR_Swap as source is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE_SUPPORT(pdHandle)                          \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptPRSwapAsSource != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Get auto accept PR_Swap as source setting */
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSource

/** @brief Check if auto accept PR_Swap as sink is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK_SUPPORT(pdHandle)                            \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptPRSwapAsSink != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Get auto accept PR_Swap as sink setting */
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSink

/** @brief Set auto accept PR_Swap as source setting */
#define PD_POLICY_SET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdHandle, val)                            \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptPRSwapAsSource = val

/** @brief Set auto accept PR_Swap as sink setting */
#define PD_POLICY_SET_AUTO_ACCEPT_PRSWAP_AS_SINK(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSink = \
        val

/* Data Role Swap Request Macros */

/** @brief Get auto request DR_Swap setting */
#define PD_POLICY_GET_AUTO_REQUEST_DRSWAP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestDRSwap

/** @brief Set auto request DR_Swap setting */
#define PD_POLICY_SET_AUTO_REQUEST_DRSWAP(pdHandle, val) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestDRSwap = val

/* Data Role Swap Accept Macros */

/** @brief Check if auto accept DR_Swap to DFP is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP_SUPPORT(pdHandle)                             \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptDRSwapToDFP != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Get auto accept DR_Swap to DFP setting */
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapToDFP

/** @brief Check if auto accept DR_Swap to UFP is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP_SUPPORT(pdHandle)                             \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptDRSwapToUFP != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Get auto accept DR_Swap to UFP setting */
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapToUFP

/** @brief Set auto accept DR_Swap to DFP setting */
#define PD_POLICY_SET_AUTO_ACCEPT_DRSWAP_AS_DFP(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapToDFP = \
        val

/** @brief Set auto accept DR_Swap to UFP setting */
#define PD_POLICY_SET_AUTO_ACCEPT_DRSWAP_AS_UFP(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapToUFP = \
        val

/* VCONN Swap Request Macros */

/** @brief Get auto request VCONN_Swap setting */
#define PD_POLICY_GET_AUTO_REQUEST_VCONNSWAP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestVConnSwap

/** @brief Set auto request VCONN_Swap setting */
#define PD_POLICY_SET_AUTO_REQUEST_VCONNSWAP(pdHandle, val) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestVConnSwap = val

/* VCONN Swap Accept Macros */

/** @brief Check if auto accept VCONN_Swap to turn on is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON_SUPPORT(pdHandle)                         \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptVconnSwapToOn != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Check if auto accept VCONN_Swap to turn off is supported */
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF_SUPPORT(pdHandle)                        \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptVconnSwapToOff != (uint8_t)kAutoRequestProcess_NotSupport)

/** @brief Get auto accept VCONN_Swap to turn on setting */
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptVconnSwapToOn

/** @brief Get auto accept VCONN_Swap to turn off setting */
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptVconnSwapToOff

/** @brief Set auto accept VCONN_Swap to turn on setting */
#define PD_POLICY_SET_AUTO_ACCEPT_VCONNSWAP_TURN_ON(pdHandle, val)                           \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptVconnSwapToOn = val

/** @brief Set auto accept VCONN_Swap to turn off setting */
#define PD_POLICY_SET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF(pdHandle, val)                          \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptVconnSwapToOff = val

/* Sink Negotiation Macro */

/** @brief Check if auto sink negotiation is supported */
#define PD_POLICY_GET_AUTO_SINK_NEGOTIATION_SUPPORT(pdHandle)                                                         \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoSinkNegotiation != \
     (uint8_t)kAutoRequestProcess_NotSupport)

#endif
