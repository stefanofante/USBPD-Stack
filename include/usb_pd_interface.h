/**
 * @file usb_pd_interface.h
 * @brief USB PD Stack Interface Definitions and API
 * 
 * @details This header defines the core USB PD stack interface including:
 *          - PD instance structure containing complete state machine context
 *          - Task event definitions for PD event-driven architecture
 *          - TCPC register cache structure for I2C optimization
 *          - Auto policy state machine for automatic PD negotiation
 *          - Connection state tracking and management
 *          - API declarations for stack initialization, task scheduling, PHY control
 * 
 *          The pd_instance_t structure is the central data container holding:
 *          - Physical layer interface (PTN5110/PTN5100 TCPC)
 *          - Timer state (bitmap arrays for 46 timers)
 *          - Message buffers (send/receive with extended message support)
 *          - Policy engine state (PSM states, auto policy, VDM handling)
 *          - Type-C connection state (CC monitoring, role tracking)
 *          - Power delivery state (PDOs, RDO, explicit contract flags)
 * 
 *          Event-driven architecture uses OSA event flags to coordinate:
 *          - Hard reset reception
 *          - PD message arrival
 *          - DPM command requests
 *          - Message send completion
 *          - Timer expiration
 *          - PHY state changes
 * 
 * @copyright Copyright 2016 - 2017 NXP. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PD_INTERFACE_H__
#define __PD_INTERFACE_H__

#include "usb_pd_i2c.h"
#include "usb_pd_spec.h"
#include "support/fsl_adapter_gpio.h"
#include "support/fsl_os_abstraction.h"
#include "usb_pd_timer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG)
#undef PD_CONFIG_CABLE_COMMUNICATION_ENABLE
#define PD_CONFIG_CABLE_COMMUNICATION_ENABLE (1U)
#endif

#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))
#undef PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE
#define PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE (1U)
#endif

/** @brief Number of secondary PSM states for VDM handling */
#define PSM_SECONDARY_STATE_COUNT (3U)

/** @brief Debug accessory role configuration */
#define PD_CONFIG_DEBUG_ACCESSORY_ROLE (CONFIG_DEBUG_ACCESSORY_DTS)

/** @brief PD task event wait time (wait forever) */
#define PD_WAIT_EVENT_TIME (osaWaitForever_c)

/** @brief Cable discovery identity retry count */
#define PD_TRY_GET_CABLE_INFO_COUNT (3U)

/* private */

/* pd_task_event_type_t - Event flags for PD task coordination */

/** @brief Hard reset received event flag */
#define PD_TASK_EVENT_RECEIVED_HARD_RESET   (0x01U)

/** @brief PD message received event flag */
#define PD_TASK_EVENT_PD_MSG                (0x02U)

/** @brief DPM command request event flag */
#define PD_TASK_EVENT_DPM_MSG               (0x04U)

/** @brief Message send complete event flag */
#define PD_TASK_EVENT_SEND_DONE             (0x08U)

/** @brief Timer timeout event flag */
#define PD_TASK_EVENT_TIME_OUT              (0x10U)

/** @brief PHY state change event flag (typo: CHAGNE) */
#define PD_TASK_EVENT_PHY_STATE_CHAGNE      (0x20U)

/** @brief Other miscellaneous event flag */
#define PD_TASK_EVENT_OTHER                 (0x40U)

/** @brief Fast role swap signal received event flag */
#define PD_TASK_EVENT_FR_SWAP_SINGAL        (0x80U)

/** @brief VBUS sink disconnect detected event flag */
#define PD_TASK_EVENT_VBUS_SINK_DISCONNECT  (0x100U)

/** @brief Configuration reset event flag */
#define PD_TASK_EVENT_RESET_CONFIGURE       (0x200U)

/** @brief External power change event flag */
#define PD_TASK_EVENT_EXTERNAL_POWER_CHANGE (0x400U)

/** @brief All events mask */
#define PD_TASK_EVENT_ALL                   (0xFFFFU)

/**
 * @brief Debug accessory role types
 * 
 * Defines the role of debug accessory per Type-C specification.
 */
typedef enum
{
    CONFIG_DEBUG_ACCESSORY_NONE = 0, /**< No debug accessory */
    CONFIG_DEBUG_ACCESSORY_TS   = 1, /**< Debug accessory with Test Support (TS) */
    CONFIG_DEBUG_ACCESSORY_DTS  = 2, /**< Debug accessory with Data Through Support (DTS) */
} pd_debug_acc_role_t;

/**
 * @brief Automatic policy state machine states
 * 
 * State machine for automatic PD policy negotiation including power role swap,
 * data role swap, and VCONN swap operations. Runs in ready state to optimize
 * power delivery configuration.
 */
typedef enum _pd_auto_policy_state
{
    PSM_RDY_EVAL_INIT = 0,          /**< Initialize auto policy evaluation */
    PSM_RDY_EVAL_GET_SNK_CAP,       /**< Get sink capabilities from partner */
    PSM_RDY_EVAL_CHECK_PARTNER_CAP, /**< Check partner capabilities */
    PSM_RDY_EVAL_SWAP_TO_SRC,       /**< Initiate power role swap to source */
    PSM_RDY_EVAL_CHECK_SWAP_TO_SRC, /**< Check swap to source result */
    PSM_RDY_EVAL_SWAP_TO_SNK,       /**< Initiate power role swap to sink */
    PSM_RDY_EVAL_CHECK_SWAP_TO_SNK, /**< Check swap to sink result */
    PSM_RDY_EVAL_DR_SWAP,           /**< Initiate data role swap */
    PSM_RDY_EVAL_CHECK_DR_SWAP,     /**< Check data role swap result */
    PSM_RDY_EVAL_VCONN_SWAP,        /**< Initiate VCONN swap */
    PSM_RDY_EVAL_CHECK_VCONN_SWAP,  /**< Check VCONN swap result */
    PSM_RDY_EVAL_IDLE,              /**< Auto policy idle state */
    PSM_RDY_DELAY_FLAG = 0x80,      /**< Delay flag before each step */
} pd_auto_policy_state_t;

/**
 * @brief Connection state enumeration
 * 
 * Indicates the stability and validity of Type-C connection.
 */
typedef enum _pd_connect_state
{
    kConnectState_NotStable,    /**< Connection not yet stable */
    kConnectState_Connected,    /**< Stably connected */
    kConnectState_Disconnected, /**< Disconnected state */
} pd_connect_state_t;

/**
 * @brief TCPC register cache structure
 * 
 * Caches frequently accessed TCPC registers to minimize I2C transactions
 * and improve performance. Organized by register functional groups.
 */
typedef struct
{
    struct
    {
        uint16_t vendor_id;         /**< Vendor ID register */
        uint16_t product_id;        /**< Product ID register */
        uint16_t device_id;         /**< Device ID register */
        uint16_t usbtypec_rev;      /**< USB Type-C revision register */
        uint16_t usbpd_rev_ver;     /**< USB PD revision/version register */
        uint16_t pd_interface_rev;  /**< PD interface revision register */
    } GLOBAL;
    struct
    {
        uint16_t alert;             /**< Alert register */
    } INTERRUPT;
    struct
    {
        uint16_t alert_mask;           /**< Alert mask register */
        uint8_t alert_extended_mask;   /**< Extended alert mask register */
        uint8_t power_status_mask;     /**< Power status mask register */
    } MASK;
    struct
    {
        uint8_t receive_detect;     /**< Receive detect register */
    } MSG_RX;
    struct
    {
        uint8_t tcpc_control;       /**< TCPC control register */
        uint8_t role_control;       /**< Role control register */
        uint8_t fault_control;      /**< Fault control register */
        uint8_t power_control;      /**< Power control register */
    } CONTROL;
    struct
    {
        uint8_t cc_status;          /**< CC status register */
        uint8_t power_status;       /**< Power status register */
        uint8_t fault_status;       /**< Fault status register */
        uint8_t extended_status;    /**< Extended status register */
    } STATUS;
    struct
    {
        uint16_t vbus_sink_disconnect_threshold; /**< VBUS sink disconnect threshold */
    } VBUS;
} pd_phy_TCPC_reg_cache_t;

/**
 * @brief PD stack instance structure
 * 
 * Complete context for a single USB PD port instance. Contains all state machine
 * variables, message buffers, timer state, configuration, and callback references.
 * This is the central data structure passed to all PD stack API functions.
 * 
 * Memory layout optimized for:
 * - Fast event-driven processing (event handle at top)
 * - Minimal I2C overhead (TCPC register cache)
 * - Efficient timer handling (bitmap arrays for 46 timers)
 * - Extended message support (up to 266-byte messages)
 * 
 * Key functional areas:
 * - Configuration: pdConfig, pdPowerPortConfig, phyInterface
 * - Event coordination: taskEventHandle, taskWaitTime
 * - Message handling: receivingData, sendingData, message state flags
 * - Timer management: timrsRunningState, timrsTimeOutState, timrsTimeValue
 * - Policy state: psmCurState, psmNewState, auto policy state
 * - Type-C connection: curConnectState, ccUsed, raPresent
 * - Power delivery: rdoRequest, partnerSourcePDOs, explicit contract flags
 */
typedef struct _pd_instance
{
#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
    void *altModeHandle; /*!< alternative mode handle */
#endif
    pd_instance_config_t *pdConfig;             /*!< PD instance configuration */
    pd_power_port_config_t *pdPowerPortConfig;  /*!< Port power configuration */
    const pd_phy_api_interface_t *phyInterface; /*!< Physical layer driver interface */
    pd_stack_callback_t pdCallback;             /*!< PD demo callback function */
    pd_power_handle_callback_t *callbackFns;    /*!< Port power control callback function */
    void *callbackParam;                        /*!< Point to an application instance */
    OSA_EVENT_HANDLE_DEFINE(taskEventHandle);   /*!< PD task event handle */
    volatile uint32_t taskWaitTime;             /*!< Used to save blocking time of PD task event */
    usb_pd_i2c_handle i2cHandle;                /*!< PD stack i2c handle */
    GPIO_HANDLE_DEFINE(gpioAlertHandle);        /*!< Alert pin handle*/
    pd_phy_TCPC_reg_cache_t tcpcRegCache;       /*!< TCPC register cache is mainly used to reduce the
                                                   number of addressing remote registers. */

    /* PD state machine */
    pd_rdo_t rdoRequest;                         /*!< Self Sink RDO */
    pd_rdo_t partnerRdoRequest;                  /*!< Partner RDO */
    pd_source_pdo_t selfOrPartnerFirstSourcePDO; /*!< Self or partner first Source PDO */
    pd_source_pdo_t partnerSourcePDOs[7];        /*!< Partner Source PDO */
    uint32_t psmCableIdentities[7];              /*!< VDO from the Discover Identity response of e-mark cable */
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    pd_sink_pdo_t portPartnerSinkPDO1; /*!< Partner Sink PDO */
#endif
    /* timr */
    volatile uint32_t timrsRunningState[(PD_MAX_TIMER_COUNT + 31) / 32]; /*!< This is a flag which indicates that the
                                                                            corresponding timer is running. */
    volatile uint32_t timrsTimeOutState[(PD_MAX_TIMER_COUNT + 31) / 32]; /*!< This is a flag which indicates that the
                                                                            corresponding timer has been timeout. */
    /* pd msg process */
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t receivingData[67];       /*!< Receiving message buffer. Maximum message length is equal to 266 bytes. */
    uint32_t receivingChunkedData[8]; /*<! This is a temporary buffer used to receive an incoming chunked message. */
    uint32_t receivingDataCache[67];  /*<! Only when the last message hasn't been processed yet, the cache is valid and
                                           is used to save the latest message. */
#else
    uint32_t receivingData[8]; /*!< Receiving message buffer. Maximum message length is equal to 32 bytes. */
    uint32_t receivingDataCache[8];
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t sendingData[67]; /*!< Sending message buffer. */
#else
    uint32_t sendingData[8]; /*!< Sending message buffer  */
#endif
    uint32_t *receivedData;        /*!< The pointer of received data which points to the receiving message buffer */
    uint32_t receivedLength;       /*!< The length of receiving message buffer is from message header to the last
                                               valid byte of this buffer. */
    uint32_t *receivingDataBuffer; /*!< The pointer of receiving buffer which points to the receiving message buffer.
                                      The RECEIVE_BUFFER from TCPC will include 0-RECEIVE_BYTE_COUNT,
                                      1-RX_BUF_FRAME_TYPE, 2-RX_BUF_HEADER_BYTE_0, 3-RX_BUF_HEADER_BYTE_1,
                                      byte[0], byte[1], ..., byte[N] */
    volatile pd_status_t receiveResult;     /*!< The result of receiving message.
                                               There may be a bad result. e.g. RECEIVE_BYTE_COUNT is equal to 0. */
    volatile pd_status_t sendingResult;     /*!< The result of sending message.
                                               There may be a bad result. e.g. Not receive a GoodCRC. */
    volatile start_of_packet_t receivedSop; /*!< Received Start Of Packet. */
    volatile start_of_packet_t sendingSop;  /*!< Sending Start Of Packet. */
    start_of_packet_t psmSoftResetSop;      /*!< Used to save the SOP* of sending SoftReset */
    volatile uint8_t pendingSOP;            /*!< The type of SOP*. Allow PD stack to receive SOP* message */
    volatile uint8_t sendingState;          /*!< 0 - no pending send; 1 - sending; 2 - send callback done */
    volatile uint8_t receiveState;          /*!< 0 - no pending receive; 1 - receiving; 2 - received data */
    volatile uint8_t occupied;              /*!< This is a flag used to indicate if this PD instance is occupied. */
    uint8_t msgId[5];              /*!< Message ID for sending message. The index correnspond to SOP*. Please refer to
                                      the the emumeration start_of_packet_t. */
    uint8_t rcvMsgId[5];           /*!< Message ID for received message. The index correnspond to SOP*. Please refer to
                                      the the emumeration start_of_packet_t. */
    uint8_t firstMsgAfterReset[5]; /*!< This is a flag indicating if the corresponding message ID is reset. The index
                                      correnspond to SOP*. Please refer to the the emumeration start_of_packet_t. */
    pd_phy_rx_result_t cacheRxResult; /*!< Save receiving cache result. */

    /* DPM commands */
    uint32_t dpmMsgBits; /*!< Each bit represents a DPM command. Please refer to the enumeration pd_command_t */
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    pd_svdm_command_param_t structuredVdmCommandParameter; /*!< Used to save initiator's srtucture VDM information */
    pd_unstructured_vdm_command_param_t unstructuredVdmCommandParameter; /*!< Used to save initiator's unstructure VDM
                                                                            information */
    uint32_t vdmExitReceived[3]; /*!< Used to save VDM header when receiving an Exit Mode command */
#endif
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    pd_command_data_param_t commandExtParam;         /*!< Used to save initiator's extended message information */
    pd_command_data_param_t commandExtParamCallback; /*!< Used to save responder's extended message information */
    uint32_t alertADO;                               /*!< Used to save initiator's Alert Data Object*/
#endif

    /* PD state machine */
    uint16_t dpmCableMaxCurrent; /*!< Used to save maximum cable current value. Uint: 10mA */
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    uint16_t chunkNumBytesReceived; /*!< Used to save the number of bytes for the current chunked extended message */
#endif
    /* timr */
    volatile uint16_t timrsTimeValue[PD_MAX_TIMER_COUNT]; /*!< Used to save the corresponding timer value */

    /* PD state machine */
    TypeCState_t curConnectState;       /*!< Current Type-C connection state. Please refer to Type-C spec V1.3  4.5.2 */
    pd_psm_state_t psmCurState;         /*!< Current Policy Engine state. Please refer to PD3.0 spec V1.2  8.3.3 */
    pd_psm_state_t psmNewState;         /*!< Newest Policy Engine state. Please refer to PD3.0 spec V1.2  8.3.3 */
    pd_psm_state_t psmInterruptedState; /*!< Policy Engine state before interrupted */
    pd_psm_state_t psmDrSwapPrevState;  /*!< Policy Engine state before Data Role Swap */
    pd_psm_state_t psmVconnSwapPrevState; /*!< Policy Engine state before VCONN Swap */
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    pd_psm_state_t psmSecondaryState[3];    /*!< Current Vendor Defined Message state */
    pd_psm_state_t psmNewSecondaryState[3]; /*!< Newest Vendor Defined Message state */
    pd_psm_state_t psmVDMBusyWaitDpmMsg;    /*!< Used to save VDM state lest PD Stack busy with handling previous VDM
                                        command loses the current VDM command from DPM request. */
#endif
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    pd_chunking_layer_state_t chunkingRXState; /*!< RX chunked state. Please refer to PD3.0 spec V1.2  6.11.2 */
    pd_chunking_layer_state_t chunkingTXState; /*!< TX chunked state. Please refer to PD3.0 spec V1.2  6.11.2 */
#endif

    /* PD state machine */
    volatile TypeCState_t connectedResult;     /*!< Type-C connection state result */
    typec_port_connect_state_t connectState;   /*!< Please refer to the enumeration typec_port_connect_state_t */
    pd_command_result_t commandEvaluateResult; /*!< Used to save evaluation result for DR Swap, PR Swap and VC Swap. */
    volatile pd_vdm_command_type_t amsVdmReplyMsg; /*!< Used to save command type of VDM response message. */
    pd_power_role_t initialPowerRole;              /*!< Initial power role before entering into USB PD communication */
    pd_power_role_t curPowerRole;                  /*!< Current power role. */
    pd_data_role_t curDataRole;                    /*!< Current data role. */
    pd_command_t commandProcessing;                /*!< Current DPM command being processed. */
    pd_cc_type_t ccUsed;                           /*!< Current communication CC pin. */
    pd_vbus_power_progress_t inProgress;           /*!< Vbus power state. */
    vbus_discharge_t vbusDischargeInProgress;      /*!< Vbus discharge state. */
    pd_vconn_role_t psmPresentlyVconnSource;       /*!< Whether to source VCONN. */
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    pd_auto_policy_state_t autoPolicyState; /*!< Automatical Policy state. */
    pd_command_result_t vconnSwapResult;    /*!< Used to save VCONN Swap result. */
    pd_command_result_t drSwapResult;       /*!< Used to save Data Swap result. */
#endif
    typec_current_val_t sinkRpVal; /*!< Used to save the current Snk.Rp value monitored by Sink */

    uint8_t dpmStateMachine;             /*!< This is flag which indicates if device policy manage run */
    volatile uint8_t commandIsInitiator; /*!< This is a flag which indicates if PD stack is DPM command initiator. */
    volatile uint8_t partnerSourcePDOsCount; /*!< Used to save the number of partner Source PDO */
    uint8_t raPresent;                       /*!< Which of CC pins presents SRC.Ra. 0 - None, 1 - CC1, 2 - CC2 */
    uint8_t psmVdmActiveModeValidMask; /*!< Which bit is set indicates which VDO in the list the Enter Mode refers to */
    uint8_t psmHardResetCount;         /*!< HardReset counter */
    uint8_t psmSendCapsCounter;        /*!< Source Capabilities counter */
    uint8_t revision;                  /*!< PD Specification Revision. The value can be PD_SPEC_REVISION_10,
                                          PD_SPEC_REVISION_20 or PD_SPEC_REVISION_30 */
    uint8_t psmCableIdentitiesDataCount; /*!< The number of VDO from the Discover Identity response of e-mark cable */
    volatile uint8_t noConnectButVBusExist; /*!< An exception state. Not detect attached but there exist Vbus. */
#if (defined(PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)) && \
    (defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE))
    uint8_t commandRetryCount; /*!< DPM command retry counter */
#endif
#if (defined PD_CONFIG_PHY_LOW_POWER_LEVEL) && (PD_CONFIG_PHY_LOW_POWER_LEVEL)
    uint8_t lowPowerState; /*!< Low power state, 0 - not in low power state, non-zero - in low power state */
#endif
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    uint8_t cableDiscoverIdentityCounter; /*!< Discover Identity Counter */
#endif
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    uint8_t chunkNumberToSend;   /*!< TX chunked state machine is sending the value of Chunk Number. */
    uint8_t chunkSentDone;       /*!< This is a flag that indicate the chunked messages have been done. */
    uint8_t chunkNumberExpected; /*!< RX chunked state machine is expecting the value of Chunk Number. */
#endif

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint8_t getBatteryCapDataBlock; /*!< Used to save the value of the Data Block from Get_Battery_Status or
                                       Get_Battery_Cap Message. */
#endif

#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
    uint8_t drSwapTriggerExitMode : 1; /*!< This is a flag indicating that DR Swap in alternate mode needs exit mode. */
#endif
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    uint8_t cableDiscoverIdentityTimeout : 1; /*!< This is a flag indicating DiscoverIdentityTimer timeout. */
#endif
    /* msg process */
    uint8_t pendingMsg : 1;                 /*!< This is a flag indicating if there exists pending message. */
    volatile uint8_t hardResetReceived : 1; /*!< This is a flag indicating HardReset received. */
    /* pd state machine */
    volatile uint8_t initializeLabel : 1;   /*!< This is a flag indicating if it is the first time to enter PD stack. */
    volatile uint8_t isConnected : 1;       /*!< This is a flag indicating if Type-C is attached state. */
    uint8_t trySNKState : 1;                /*!< This is a flag indicating if Type-C enter TrySnk state.  */
    uint8_t psmGotoMinTx : 1;               /*!< This is a flag indicating if DPM requests GotoMin. */
    uint8_t psmGotoMinRx : 1;               /*!< This is a flag indicating if GotoMin received. */
    uint8_t enterTrySNKFromPoweredAcc : 1;  /*!< This is a flag indicating if Type-C transition to TrySnk state from
                                               PoweredAccessory state. */
    uint8_t cc1Monitor : 1;                 /*!< This is a flag indicating if Type-C state machine
                                               is monitoring CC1 state. */
    uint8_t cc2Monitor : 1;                 /*!< This is a flag indicating if Type-C state machine
                                               is monitoring CC2 state. */
    uint8_t psmHardResetNeedsVSafe0V : 1;   /*!< This is a flag indicating that vSafe0V is needed when Policy Engine
                                               state machine enter PE_SRC_Transition_to_default or
                                               PE_SNK_Transition_to_default state. */
    uint8_t psmPresentlyPdConnected : 1;    /*!< This is a flag indicating that connection Sink can respond
                                               PD message. */
    uint8_t psmPreviouslyPdConnected : 1;   /*!< This is a flag indicating that connection Sink ever responded
                                                 PD message. */
    uint8_t psmCablePlugResetNeeded : 1;    /*!< This is a flag indicating if SoftReset for cable is needed during
                                              Data Role Swap, Power Role Swap or Fast Role Swap. */
    uint8_t psmSnkReceiveRdoWaitRetry : 1;  /*!< This is a flag indicating Sink received a Wait message after sending
                                               the Request message when a PD explict contract has been established. */
    uint8_t psmExplicitContractExisted : 1; /*!< This is a flag indicating if PD explict contract is established. */
    volatile uint8_t asmHardResetSnkProcessing : 1; /*!< This is a flag indicating if Sink is in HardReset progress. */
    uint8_t unchunkedFeature : 1;        /*!< This is a flag indicating if PD stack support Unchunked message. */
    volatile uint8_t alertWaitReply : 1; /*!< This is a flag indicating that Alert initiator is waiting for a reply. */
    volatile uint8_t fr5VOpened : 1;     /*!< This is a flag indicating if 5V Vbus is sourced during Fast Role Swap. */
    volatile uint8_t enableReceive : 1;  /*!< Enable PD stack to receive message. */
    volatile uint8_t enterSrcFromSwap : 1; /*!< From Sink to Source by Power Role Swap or Fast Role Swap. */
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    volatile uint8_t commandSrcOwner : 1; /*!< This is a flag indicating if command initiator is the owner of source. */
    volatile uint8_t powerNegotiationInitiator : 1; /*!< Initiate power negotiation AMS or not */
#endif
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    volatile uint8_t frSwapReceived : 1;       /*!< This is a flag indicating that FR_Swap message is received. */
    volatile uint8_t frSignaledWaitFrSwap : 1; /*!< Be waiting for FR_Swap message after signalling FR_Swap signal. */
    volatile uint8_t frsEnabled : 1;           /*!< Enable Fast Role Swap */
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    uint8_t dumpResponse : 1;                               /*!< Used during auto policy process, the bit is set
                                                                 indicating the current AMS doesn't need to call
                                                                 application of callback function. */
    uint8_t swapToSnkSrcCapReceived : 1;                    /*!< Source Capbilities message has been received during
                                                               Power Role Swap from Source to Sink */
    volatile uint8_t rdySeenExtPower : 1;                   /*!< External power state. 0 - not exist, 1 - exsit */
    volatile uint8_t portPartnerVconnSwapToOffRejected : 1; /*!< This is a flag indicating that port partner reject the
                                                               initiator to turn off VCONN during VCONN Swap. */
    volatile uint8_t portPartnerVconnSwapToOnRejected : 1;  /*!< This is a flag indicating that port partner reject the
                                                               initiator to turn on VCONN during VCONN Swap. */
    volatile uint8_t portPartnerDrSwapToUFPRejected : 1;    /*!< This is a flag indicating that port partner reject the
                                                               initiator to swap to UFP during Data Role Swap. */
    volatile uint8_t portPartnerDrSwapToDFPRejected : 1;    /*!< This is a flag indicating that port partner reject the
                                                               initiator to swap to DFP during Data Role Swap. */
    volatile uint8_t portPartnerPrSwapToSinkRejected : 1;   /*!< This is a flag indicating that port partner reject the
                                                               initiator to swap to Sink during Power Role Swap. */
    volatile uint8_t portPartnerPrSwapToSourceRejected : 1; /*!< This is a flag indicating that port partner reject the
                                                               initiator to swap to Sink during Power Role Swap. */
#endif
#if (defined(__DSC__) && defined(__CW__))
    volatile uint8_t pdSetEvent : 1; /*!< DSC doesn't support low level interrupt, need to a
                                          variable to record event */
#endif
} pd_instance_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/* usb_pd_interface */

/**
 * @brief Check if PD stack has pending events
 * 
 * @param[in] pdInstance PD instance pointer
 * @return 1 if events are pending, 0 if no events
 */
uint8_t PD_StackHasPendingEvent(pd_instance_t *pdInstance);

/**
 * @brief Set event flag for PD task
 * 
 * Posts an event to the PD task event handle to trigger state machine processing.
 * Used throughout the stack to signal message reception, timer expiration, etc.
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] event Event flag bits (see PD_TASK_EVENT_* defines)
 */
void PD_StackSetEvent(pd_instance_t *pdInstance, uint32_t event);

/**
 * @brief Control physical layer operations
 * 
 * Generic PHY control interface for operations like enabling interrupts,
 * updating state, controlling VBUS/VCONN, etc.
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] control PHY control command
 * @param[in,out] param Command-specific parameter
 * @return kStatus_PD_Success on success, error code otherwise
 */
pd_status_t PD_PhyControl(pd_instance_t *pdInstance, pd_phy_control_t control, void *param);

/**
 * @brief Blocking microsecond delay
 * 
 * Busy-wait delay for short timing requirements. Weak function that can be
 * overridden with platform-specific implementation.
 * 
 * @param[in] us Delay duration in microseconds
 */
void PD_WaitUsec(uint32_t us);

/* usb_pd_connect */

/**
 * @brief Set VBUS power state progress
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] state VBUS power progress state
 */
void PD_ConnectSetPowerProgress(pd_instance_t *pdInstance, pd_vbus_power_progress_t state);

/**
 * @brief Get initial Type-C role state
 * 
 * @param[in] pdInstance PD instance pointer
 * @return Initial Type-C state based on configuration
 */
TypeCState_t PD_ConnectGetInitRoleState(pd_instance_t *pdInstance);

/**
 * @brief Initialize Type-C role state machine
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] errorRecovery 1 if initializing from error recovery, 0 otherwise
 */
void PD_ConnectInitRole(pd_instance_t *pdInstance, uint8_t errorRecovery);

/**
 * @brief Set power role after power role swap
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] powerRole New power role (source/sink)
 */
void PD_ConnectSetPRSwapRole(pd_instance_t *pdInstance, pd_power_role_t powerRole);

/**
 * @brief Handle alternate mode entry failure
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] pdConnected 1 if PD connected, 0 otherwise
 */
void PD_ConnectAltModeEnterFail(pd_instance_t *pdInstance, uint8_t pdConnected);

/**
 * @brief Get connection state
 * 
 * @param[in] pdInstance PD instance pointer
 * @return Connection state (stable/connected/disconnected)
 */
pd_connect_state_t PD_ConnectState(pd_instance_t *pdInstance);

/**
 * @brief Check and update connection state
 * 
 * @param[in] pdInstance PD instance pointer
 * @return Updated connection state
 */
pd_connect_state_t PD_ConnectCheck(pd_instance_t *pdInstance);

/**
 * @brief Get current Type-C state machine state
 * 
 * @param[in] pdInstance PD instance pointer
 * @return Current Type-C state
 */
TypeCState_t PD_ConnectGetStateMachine(pd_instance_t *pdInstance);

/* usb_pd_msg */

/**
 * @brief Reset message handling subsystem
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReset(pd_instance_t *pdInstance);

/**
 * @brief Notify message send completion
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] result Send result status
 */
void PD_MsgSendDone(pd_instance_t *pdInstance, pd_status_t result);

/**
 * @brief Send PD message
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] sop Start of packet type (SOP/SOP'/SOP'')
 * @param[in] msgType Message type
 * @param[in] dataLength Data object count
 * @param[in] dataBuffer Pointer to data objects
 * @return kStatus_PD_Success on success, error code otherwise
 */
pd_status_t PD_MsgSend(
    pd_instance_t *pdInstance, start_of_packet_t sop, message_type_t msgType, uint32_t dataLength, uint8_t *dataBuffer);

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
/**
 * @brief Send extended PD message (up to 266 bytes)
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] sop Start of packet type
 * @param[in] extMsgType Extended message type
 * @param[in] dataLength Total data length
 * @param[in] dataBuffer Pointer to data buffer
 * @param[in] dataSize Size of this chunk
 * @param[in] requestChunk 1 if requesting chunk, 0 otherwise
 * @param[in] chunkNumber Current chunk number
 * @return kStatus_PD_Success on success, error code otherwise
 */
pd_status_t PD_MsgSendExtendedMsg(pd_instance_t *pdInstance,
                                  start_of_packet_t sop,
                                  message_type_t extMsgType,
                                  uint32_t dataLength,
                                  uint8_t *dataBuffer,
                                  uint16_t dataSize,
                                  uint8_t requestChunk,
                                  uint8_t chunkNumber);
#endif /* PD_CONFIG_EXTENDED_MSG_SUPPORT */

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) ||        \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
/**
 * @brief Send structured VDM (Vendor Defined Message)
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] sop Start of packet type
 * @param[in] reponseVdmHeader VDM header structure
 * @param[in] count Number of VDOs (data objects)
 * @param[in] vdos Pointer to VDO array
 * @return kStatus_PD_Success on success, error code otherwise
 */
pd_status_t PD_MsgSendStructuredVDM(pd_instance_t *pdInstance,
                                    start_of_packet_t sop,
                                    pd_structured_vdm_header_t reponseVdmHeader,
                                    uint8_t count,
                                    uint32_t *vdos);
#endif /* PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE */

/**
 * @brief Handle hard reset reception
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReceivedHardReset(pd_instance_t *pdInstance);

/**
 * @brief Send hard reset or cable reset
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] hardResetOrCableReset 1 for hard reset, 0 for cable reset
 */
void PD_MsgSendHardOrCableReset(pd_instance_t *pdInstance, uint8_t hardResetOrCableReset);

/**
 * @brief Process received message from PHY
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] rxResult RX result structure from PHY
 */
void PD_MsgReceived(pd_instance_t *pdInstance, pd_phy_rx_result_t *rxResult);

/**
 * @brief Stop message reception
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgStopReceive(pd_instance_t *pdInstance);

/**
 * @brief Start message reception
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgStartReceive(pd_instance_t *pdInstance);

/**
 * @brief Check for received messages
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReceive(pd_instance_t *pdInstance);

/**
 * @brief Get receive result status
 * 
 * @param[in] pdInstance PD instance pointer
 * @return Receive result status
 */
uint8_t PD_MsgGetReceiveResult(pd_instance_t *pdInstance);

/**
 * @brief Check if message receive is pending
 * 
 * @param[in] pdInstance PD instance pointer
 * @return 1 if receive pending, 0 otherwise
 */
uint8_t PD_MsgRecvPending(pd_instance_t *pdInstance);

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
/**
 * @brief Check if source can start AMS (Atomic Message Sequence)
 * 
 * @param[in] pdInstance PD instance pointer
 * @return 1 if can start, 0 otherwise
 */
uint8_t PD_MsgSrcStartCommand(pd_instance_t *pdInstance);

/**
 * @brief End source AMS command
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgSrcEndCommand(pd_instance_t *pdInstance);

/**
 * @brief Check if sink can start command (collision avoidance)
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] command DPM command to check
 * @return 1 if can start, 0 otherwise
 */
uint8_t PD_MsgSnkCheckStartCommand(pd_instance_t *pdInstance, pd_command_t command);
#endif /* PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE */

/* usb_pd_policy */

/**
 * @brief Check if VBUS is at vSafe0V
 * 
 * @param[in] pdInstance PD instance pointer
 * @return 1 if at vSafe0V, 0 otherwise
 */
uint8_t PD_PsmCheckVsafe0V(pd_instance_t *pdInstance);

/**
 * @brief Check if VBUS is at vSafe5V
 * 
 * @param[in] pdInstance PD instance pointer
 * @return 1 if at vSafe5V, 0 otherwise
 */
uint8_t PD_PsmCheckVsafe5V(pd_instance_t *pdInstance);

/**
 * @brief Wait for timer with event checking
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] timrName Timer identifier
 * @param[in] timrTime Timer duration in ms
 * @param[in] checkEvent Event mask to check during wait
 * @return Timer remaining time or 0 if timeout/event occurred
 */
uint16_t PD_PsmTimerWait(pd_instance_t *pdInstance, tTimer_t timrName, uint16_t timrTime, uint32_t checkEvent);

/**
 * @brief Process port task events
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] eventSet Event flags that are set
 */
void PD_PortTaskEventProcess(pd_instance_t *pdInstance, uint32_t eventSet);

/**
 * @brief Main PD stack state machine
 * 
 * Executes one iteration of Type-C connection detection and PD policy engine.
 * Should be called from PD task when events are pending.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_StackStateMachine(pd_instance_t *pdInstance);

/**
 * @brief Control VBUS discharge
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] enable 1 to enable discharge, 0 to disable
 */
void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable);

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
/**
 * @brief Control VCONN discharge
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] enable 1 to enable discharge, 0 to disable
 */
void PD_DpmDischargeVconn(pd_instance_t *pdInstance, uint8_t enable);

/**
 * @brief Control VCONN output
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] enable 1 to enable VCONN, 0 to disable
 */
void PD_DpmSetVconn(pd_instance_t *pdInstance, uint8_t enable);
#endif /* PD_CONFIG_VCONN_SUPPORT */

/**
 * @brief Alternate mode DPM callback
 * 
 * Called by alternate mode layer to notify DPM of events.
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] event DPM callback event type
 * @param[in] param Event-specific parameter
 */
void PD_DpmAltModeCallback(pd_handle pdHandle, pd_dpm_callback_event_t event, void *param);

#endif /* __PD_INTERFACE_H__ */
