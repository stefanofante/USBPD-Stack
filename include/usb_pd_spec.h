/**
 * @file usb_pd_spec.h
 * @brief USB Power Delivery Specification Definitions
 * 
 * @details Defines constants, enumerations, and structures per USB PD specification.
 *          Covers USB PD 2.0/3.0 message formats, data objects, and protocol structures.
 * 
 *          Message header format (16-bit):
 *          - Bits 0-4: Message type
 *          - Bit 5: Port data role (UFP/DFP)
 *          - Bits 6-7: Spec revision (PD 2.0/3.0)
 *          - Bit 8: Port power role (Source/Sink)
 *          - Bits 9-11: Message ID
 *          - Bits 12-14: Number of data objects
 *          - Bit 15: Extended message flag
 * 
 *          Extended message header (16-bit):
 *          - Bits 0-8: Data size (0-260 bytes)
 *          - Bit 10: Request chunk
 *          - Bits 11-14: Chunk number
 *          - Bit 15: Chunked flag
 * 
 *          Message types:
 *          - Control messages (0x00-0x1F): No data objects
 *          - Data messages (0x80-0x8F): 1-7 data objects
 *          - Extended messages (0xC0-0xCF): Chunked data up to 260 bytes
 * 
 *          Key data objects:
 *          - Power Data Objects (PDO): Source/Sink capabilities
 *          - Request Data Object (RDO): Power request
 *          - Vendor Defined Objects (VDO): Structured/unstructured VDM
 *          - Battery/Alert/PPS objects: PD 3.0 extended features
 * 
 * @copyright Copyright 2016 - 2017 NXP. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PD_SPEC_H__
#define __PD_SPEC_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Message Header Bit Field Definitions */

/** @brief Number of data objects position in message header */
#define PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS  (12U)
/** @brief Number of data objects mask in message header */
#define PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_MASK (0x7000u)
/** @brief Port power role position in message header */
#define PD_MSG_HEADER_PORT_POWER_ROLE_POS         (8U)
/** @brief Port power role mask in message header */
#define PD_MSG_HEADER_PORT_POWER_ROLE_MASK        (0x0100u)
/** @brief Spec revision position in message header */
#define PD_MSG_HEADER_SPEC_REV_POS                (6U)
/** @brief Port data role position in message header */
#define PD_MSG_HEADER_PORT_DATA_ROLE_POS          (5U)
/** @brief Port data role mask in message header */
#define PD_MSG_HEADER_PORT_DATA_ROLE_MASK         (0x0020u)
/** @brief Message type position in message header */
#define PD_MSG_HEADER_MESSAGE_TYPE_POS            (0U)
/** @brief Message type mask in message header */
#define PD_MSG_HEADER_MESSAGE_TYPE_MASK           (0x001Fu)
/** @brief Extended message flag mask */
#define PD_MSG_HEADER_EXTENDED_MASK               (0x8000u)
/** @brief Extended message flag position */
#define PD_MSG_HEADER_EXTENDED_POS                (15U)

/* Extended Message Header Bit Field Definitions */

/** @brief Chunked flag mask in extended header */
#define PD_MSG_EXT_HEADER_CHUNKED_MASK       (0x8000u)
/** @brief Chunked flag position in extended header */
#define PD_MSG_EXT_HEADER_CHUNKED_POS        (15U)
/** @brief Chunk number mask in extended header */
#define PD_MSG_EXT_HEADER_CHUNK_NUMBER_MASK  (0x7800u)
/** @brief Chunk number position in extended header */
#define PD_MSG_EXT_HEADER_CHUNK_NUMBER_POS   (11U)
/** @brief Request chunk flag mask in extended header */
#define PD_MSG_EXT_HEADER_REQUEST_CHUNK_MASK (0x0400u)
/** @brief Request chunk flag position in extended header */
#define PD_MSG_EXT_HEADER_REQUEST_CHUNK_POS  (10U)
/** @brief Data size mask in extended header (0-260 bytes) */
#define PD_MSG_EXT_HEADER_DATA_SIZE_MASK     (0x01FFu)
/** @brief Data size position in extended header */
#define PD_MSG_EXT_HEADER_DATA_SIZE_POS      (0U)

/* Message Type Classification Masks */

/** @brief Control message type mask (no data objects) */
#define PD_MSG_CONTROL_TYPE_MASK (0x00u)
/** @brief Data message type mask (1-7 data objects) */
#define PD_MSG_DATA_TYPE_MASK    (0x80u)
/** @brief Extended message type mask (chunked data) */
#define PD_MSG_EXT_TYPE_MASK     (0xC0u)
/** @brief Message type bits mask */
#define PD_MSG_TYPE_BITS_MASK    (0xC0u)
/** @brief Message type value mask */
#define PD_MSG_TYPE_VALUE_MASK   (0x3Fu)

/**
 * @brief USB PD message types enumeration
 * 
 * Complete list of USB PD 2.0/3.0 message types including:
 * - Control messages (0x01-0x1F): Protocol control, no data
 * - Data messages (0x81-0x8F): Carry 1-7 data objects
 * - Extended messages (0xC1-0xCF): PD 3.0 chunked messages
 */
typedef enum _message_type
{
    /* Control Messages */
    kPD_MsgInvalid              = 0x00u, /**< Invalid/undefined message */
    kPD_MsgGoodCRC              = 0x01u, /**< GoodCRC acknowledgment */
    kPD_MsgGotoMin              = 0x02u, /**< Transition to minimum power */
    kPD_MsgAccept               = 0x03u, /**< Accept request/command */
    kPD_MsgReject               = 0x04u, /**< Reject request/command */
    kPD_MsgPing                 = 0x05u, /**< Ping message */
    kPD_MsgPsRdy                = 0x06u, /**< Power supply ready */
    kPD_MsgGetSourceCap         = 0x07u, /**< Get source capabilities */
    kPD_MsgGetSinkCap           = 0x08u, /**< Get sink capabilities */
    kPD_MsgDrSwap               = 0x09u, /**< Data role swap request */
    kPD_MsgPrSwap               = 0x0Au, /**< Power role swap request */
    kPD_MsgVconnSwap            = 0x0Bu, /**< VCONN swap request */
    kPD_MsgWait                 = 0x0Cu, /**< Wait response */
    kPD_MsgSoftReset            = 0x0Du, /**< Soft reset */
    kPD_MsgReserved1            = 0x0Eu, /**< Reserved */
    kPD_MsgReserved2            = 0x0Fu, /**< Reserved */
    kPD_MsgNotSupported         = 0x10u, /**< Not supported (PD 3.0) */
    kPD_MsgGetSourceCapExtended = 0x11u, /**< Get source cap extended */
    kPD_MsgGetStatus            = 0x12u, /**< Get status */
    kPD_MsgFrSwap               = 0x13u, /**< Fast role swap */
    kPD_MsgGetPpsStatus         = 0x14u, /**< Get PPS status */
    
    /* Data Messages */
    kPD_MsgSourceCapabilities   = (PD_MSG_DATA_TYPE_MASK | 0x01u), /**< Source capabilities */
    kPD_MsgRequest              = (PD_MSG_DATA_TYPE_MASK | 0x02u), /**< Power request */
    kPD_MsgBIST                 = (PD_MSG_DATA_TYPE_MASK | 0x03u), /**< BIST mode */
    kPD_MsgSinkCapabilities     = (PD_MSG_DATA_TYPE_MASK | 0x04u), /**< Sink capabilities */
    kPD_MsgBatteryStatus        = (PD_MSG_DATA_TYPE_MASK | 0x05u), /**< Battery status */
    kPD_MsgAlert                = (PD_MSG_DATA_TYPE_MASK | 0x06u), /**< Alert message */
    kPD_MsgVendorDefined        = (PD_MSG_DATA_TYPE_MASK | 0x0Fu), /**< Vendor defined message */

    /* Extended Messages (PD 3.0) */
    kPD_MsgSourceCapExtended      = (PD_MSG_EXT_TYPE_MASK | 0x01u), /**< Source cap extended */
    kPD_MsgStatus                 = (PD_MSG_EXT_TYPE_MASK | 0x02u), /**< Status extended */
    kPD_MsgGetBatteryCap          = (PD_MSG_EXT_TYPE_MASK | 0x03u), /**< Get battery cap */
    kPD_MsgGetBatteryStatus       = (PD_MSG_EXT_TYPE_MASK | 0x04u), /**< Get battery status */
    kPD_MsgBatteryCapabilities    = (PD_MSG_EXT_TYPE_MASK | 0x05u), /**< Battery capabilities */
    kPD_MsgGetManufacturerInfo    = (PD_MSG_EXT_TYPE_MASK | 0x06u), /**< Get manufacturer info */
    kPD_MsgManufacturerInfo       = (PD_MSG_EXT_TYPE_MASK | 0x07u), /**< Manufacturer info */
    kPD_MsgSecurityRequest        = (PD_MSG_EXT_TYPE_MASK | 0x08u), /**< Security request */
    kPD_MsgSecurityResponse       = (PD_MSG_EXT_TYPE_MASK | 0x09u), /**< Security response */
    kPD_MsgFirmwareUpdateRequest  = (PD_MSG_EXT_TYPE_MASK | 0x0Au), /**< Firmware update request */
    kPD_MsgFirmwareUpdaetResponse = (PD_MSG_EXT_TYPE_MASK | 0x0Bu), /**< Firmware update response */
    kPD_MsgPpsStatus              = (PD_MSG_EXT_TYPE_MASK | 0x0Cu), /**< PPS status */
} message_type_t;

/**
 * @brief Extended message types enumeration
 * 
 * Simplified enumeration for PD 3.0 extended message types.
 */
typedef enum _extended_message_type
{
    kPD_ExtMsgSourceCapExtended = 1u, /**< Source capabilities extended */
    kPD_ExtMsgStatus,                 /**< Status extended */
    kPD_ExtMsgGetBatteryCap,          /**< Get battery capabilities */
    kPD_ExtMsgGetBatteryStatus,       /**< Get battery status */
    kPD_ExtMsgBatteryCapabilities,    /**< Battery capabilities */
    kPD_ExtMsgGetManufacturerInfo,    /**< Get manufacturer info */
    kPD_ExtmsgManufacturerInfo,       /**< Manufacturer info */
    kPD_ExtMsgSecurityRequest,        /**< Security request */
    kPD_ExtMsgSecurityResponse,       /**< Security response */
    kPD_ExtMsgFirmwareUpdateRequest,  /**< Firmware update request */
    kPD_ExtMsgFirmwareUpdaetResponse, /**< Firmware update response */
} extended_message_type_t;

/**
 * @brief Policy Engine state machine states
 * 
 * Enumeration of all Policy Engine states per USB PD specification.
 * Includes source/sink states, power/data role swap states, VCONN swap,
 * alternate mode, structured/unstructured VDM, BIST, and extended message states.
 */
typedef enum _pd_psm_state
{
    PSM_UNKNOWN                      = 0, /* Internal state */
    PSM_IDLE                         = 1, /* Internal state */
    PSM_PE_SRC_STARTUP               = 2,
    PSM_PE_SRC_DISCOVERY             = 3,
    PSM_PE_SRC_SEND_CAPABILITIES     = 4,
    PSM_PE_SRC_NEGOTIATE_CAPABILITY  = 5,
    PSM_PE_SRC_TRANSITION_SUPPLY     = 6,
    PSM_PE_SRC_READY                 = 7,
    PSM_PE_SRC_DISABLED              = 8,
    PSM_PE_SRC_CAPABILITY_RESPONSE   = 9,
    PSM_PE_SRC_WAIT_NEW_CAPABILITIES = 10,

    PSM_HARD_RESET                   = 11, /* Source and Sink */
    PSM_PE_SRC_TRANSITION_TO_DEFAULT = 12,
    PSM_PE_SRC_GIVE_SOURCE_CAP       = 13,
    PSM_PE_SRC_GET_SINK_CAP          = 14,

    PSM_PE_DR_SRC_GET_SOURCE_CAP = 15,
    PSM_PE_DR_SRC_GIVE_SINK_CAP  = 16,

    PSM_SEND_SOFT_RESET = 17, /* source and sink */
    PSM_SOFT_RESET      = 18, /* source and sink */

    PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP,
    PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP,
    PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF,
    PSM_PE_PRS_SRC_SNK_ASSERT_RD,
    PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON,
    PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP,
    PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP,

    PSM_PE_SNK_STARTUP,
    PSM_PE_SNK_DISCOVERY,
    PSM_PE_SNK_WAIT_FOR_CAPABILITIES,
    PSM_PE_SNK_EVALUATE_CAPABILITY,
    PSM_PE_SNK_SELECT_CAPABILITY,
    PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT,
    PSM_PE_SNK_TRANSITION_SINK,
    PSM_PE_SNK_READY,
    PSM_PE_SNK_TRANSITION_TO_DEFAULT,
    PSM_PE_SNK_GIVE_SINK_CAP,
    PSM_PE_SNK_GET_SOURCE_CAP,

    PSM_PE_DR_SNK_GET_SINK_CAP,
    PSM_PE_DR_SNK_GIVE_SOURCE_CAP,

    PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT,
    PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP,
    PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP,
    PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF,
    PSM_PE_PRS_SNK_SRC_ASSERT_RP,
    PSM_PE_PRS_SNK_SRC_SOURCE_ON,
    PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP,
    PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP,

    /* BIST STATES */
    PSM_PE_BIST_CARRIER_MODE_2,
    PSM_PE_BIST_TEST_DATA_MODE,
    /* New States */
    PSM_CHECK_ASYNC_RX, /* Internal state: Cover the case where an unexpected packet was received during transmit, check
                         */
    /* for valid packets. */
    PSM_CHECK_SINK_SOURCE_CAP_RX, /* Internal state: cover the case where an unexpected packet is received in a sink
                                     state. */
    PSM_BYPASS,                   /* Internal state: PD operation bypassed */
    /* Type-C additions */

    PSM_PE_DRS_EVALUATE_DR_SWAP,
    PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT,
    PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_ALT_MODE_EXIT,
    PSM_PE_DRS_REJECT_DR_SWAP,
    PSM_PE_DRS_SEND_DR_SWAP,
    PSM_PE_DRS_ACCEPT_DR_SWAP,
    PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP,

    PSM_EXIT_TO_ERROR_RECOVERY,
    PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,

    /* NOTE: unattached UFP, DFP states are return to CLP */
    /* Type-C VCONN Swap */
    PSM_PE_VCS_SEND_SWAP,
    PSM_PE_VCS_EVALUATE_SWAP,
    PSM_PE_VCS_ACCEPT_SWAP,
    PSM_PE_VCS_REJECT_SWAP,
    PSM_PE_VCS_WAIT_FOR_VCONN,
    PSM_PE_VCS_TURN_ON_VCONN,
    PSM_PE_VCS_TURN_OFF_VCONN,
    PSM_PE_VCS_SEND_PS_RDY,

    /* UFP VDM */
    PSM_PE_UFP_VDM_GET_IDENTITY,
    PSM_PE_UFP_VDM_SEND_IDENTITY,
    PSM_PE_UFP_VDM_GET_SVIDS,
    PSM_PE_UFP_VDM_SEND_SVIDS,
    PSM_PE_UFP_VDM_GET_MODES,
    PSM_PE_UFP_VDM_SEND_MODES,
    PSM_PE_UFP_VDM_EVALUATE_MODE_ENTRY,
    PSM_PE_UFP_VDM_MODE_ENTRY_ACK,
    PSM_PE_UFP_VDM_MODE_ENTRY_NAK,
    PSM_PE_UFP_VDM_MODE_EXIT,
    PSM_PE_UFP_VDM_MODE_EXIT_ACK,
    PSM_PE_UFP_VDM_MODE_EXIT_NAK,
    /* UFP VDM Attention */
    NO_PE_UFP_VDM_ATTENTION_REQUEST,
    /* DFP to UFP VDM Discover Identity */
    PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST,
    PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED,
    PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED,
    PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT,

    /* DFP to Cable Plug VDM Discover Identity */
    PSM_PE_DFP_CBL_VDM_IDENTITY_REQUEST,
    DEP_PE_DFP_CBL_VDM_IDENTITY_ACKED,
    DEP_PE_DFP_CBL_VDM_IDENTITY_NAKED,

    /* DFP VDM Discover SVIDs */
    PSM_PE_DFP_VDM_SVIDS_REQUEST,
    PSM_PE_DFP_VDM_SVIDS_ACKED,
    PSM_PE_DFP_VDM_SVIDS_NAKED,
    /* DFP VDM Discover Modes */
    PSM_PE_DFP_VDM_MODES_REQUEST,
    PSM_PE_DFP_VDM_MODES_ACKED,
    PSM_PE_DFP_VDM_MODES_NAKED,
    /* DFP VDM Mode Entry */
    PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST,
    PSM_PE_DFP_VDM_MODE_ENTRY_ACKED,
    PSM_PE_DFP_VDM_MODE_ENTRY_NAKED,
    /* DFP VDM Mode Exit */
    PSM_PE_DFP_VDM_MODE_EXIT_REQUEST,
    PSM_PE_DFP_VDM_MODE_EXIT_ACKED,
    PSM_PE_DFP_VDM_MODE_EXIT_HARD_RESET,

    /* Source Startup VDM Discover Identity */
    PSM_PE_SRC_VDM_IDENTITY_REQUEST,
    PSM_PE_SRC_VDM_IDENTITY_ACKED,
    PSM_PE_SRC_VDM_IDENTITY_NAKED,

    /* DFP VDM Attention */
    PSM_PE_DFP_VDM_ATTENTION_REQUEST,

    PSM_PE_DFP_CBL_SEND_SOFT_RESET,
    PSM_PE_DFP_CBL_SEND_CABLE_RESET,
    PSM_PE_UFP_CBL_SEND_SOFT_RESET,

    PSM_PE_SRC_HARD_RESET_RECEIVED,

    PSM_INTERRUPTED_REQUEST,

    PSM_PE_DFP_CBL_VDM_SVIDS_REQUEST,
    PSM_PE_DFP_CBL_VDM_MODES_REQUEST,
    PSM_PE_DFP_CBL_P_VDM_MODE_ENTRY_REQUEST,
    PSM_PE_DFP_CBL_PP_VDM_MODE_ENTRY_REQUEST,
    PSM_PE_DFP_CBL_P_VDM_MODE_EXIT_REQUEST,
    PSM_PE_DFP_CBL_PP_VDM_MODE_EXIT_REQUEST,

    PSM_PE_DFP_UFP_VDM_BUSY,

    PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET,
    PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY,

    PSM_INTERRUPTED_VDM_RESPONSE,

    /* vendor structured vdm */
    PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST,
    PSM_PE_VENDOR_STRUCTURED_VDM_ACKED,
    PSM_PE_VENDOR_STRUCTURED_VDM_NAKED,

    /* unstructured vdm */
    PSM_PD_SEND_UNSTRUCTURED_VDM,

    /* get source extended cap */
    PE_SNK_GET_SOURCE_CAP_EXT,
    PE_DR_SRC_GET_SOURCE_CAP_EXT,
    PE_SRC_GIVE_SOURCE_CAP_EXT,
    PE_DR_SNK_GIVE_SOURCE_CAP_EXT,

    /* get status */
    PE_SNK_Get_Source_Status,
    PE_SRC_Give_Source_Status,
    PE_SRC_Get_Sink_Status,
    PE_SNK_Give_Sink_Status,

    /* get pps status */
    PE_SNK_Get_PPS_Status,
    PE_SRC_Give_PPS_Status,

    /* get battery cap */
    PE_Get_Battery_Cap,
    PE_Give_Battery_Cap,

    /* get battery status */
    PE_Get_Battery_Status,
    PE_Give_Battery_Status,

    /* get manufacturer info */
    PE_Get_Manufacturer_Info,
    PE_Give_Manufacturer_Info,

    /* security request */
    PE_Send_Security_Request,
    PE_Send_Security_Response,
    PE_Security_Response_Received,

    /* alert */
    PE_SRC_Send_Source_Alert,
    PE_SNK_Source_Alert_Received,
    PE_SNK_Send_Sink_Alert,
    PE_SRC_Sink_Alert_Received,

    /* fast role swap */
    PE_FRS_SRC_SNK_CC_Signal,
    PE_FRS_SRC_SNK_Evaluate_Swap,
    PE_FRS_SRC_SNK_Accept_Swap,
    PE_FRS_SRC_SNK_Transition_to_off,
    PE_FRS_SRC_SNK_Assert_Rd,
    PE_FRS_SRC_SNK_Wait_Source_on,

    PE_FRS_SNK_SRC_Send_Swap,
    PE_FRS_SNK_SRC_Transition_to_off,
    PE_FRS_SNK_SRC_Vbus_Applied,
    PE_FRS_SNK_SRC_Assert_Rp,
    PE_FRS_SNK_SRC_Source_on,

    PE_PSM_STATE_ROLE_RDY_STATE,
    PE_PSM_STATE_NO_CHANGE,
} pd_psm_state_t;

/**
 * @brief Type-C state machine states
 * 
 * Enumeration of all Type-C connection states including Unattached, Attached,
 * Try.SRC/Try.SNK, audio/debug accessories, dead battery, and toggle modes.
 */
typedef enum
{
    TYPEC_DISABLED        = 0,
    TYPEC_ERROR_RECOVERY  = 1,
    TYPEC_UNATTACHED_SRC  = 2,
    TYPEC_UNATTACHED_SNK  = 3,
    TYPEC_ATTACH_WAIT_SRC = 4,
    TYPEC_ATTACH_WAIT_SNK = 5,
    TYPEC_ATTACHED_SRC    = 6,
    TYPEC_ATTACHED_SNK    = 7,

    TYPEC_TRY_SRC               = 8,
    TYPEC_TRY_WAIT_SNK          = 9,
    TYPEC_TRY_SNK               = 10,
    TYPEC_TRY_WAIT_SRC          = 11,
    TYPEC_AUDIO_ACCESSORY       = 12,
    TYPEC_UNATTACHED_ACCESSORY  = 13,
    TYPEC_ATTACH_WAIT_ACCESSORY = 14,
    TYPEC_POWERED_ACCESSORY     = 15,
    TYPEC_UNSUPPORTED_ACCESSORY = 16,

    TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC = 17,
    TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC   = 18,
    TYPEC_DEBUG_ACCESSORY_SNK            = 19,
    TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK   = 20,
    TYPEC_TOGGLE_SRC_FIRST               = 21,
    TYPEC_TOGGLE_SNK_FIRST               = 22,
    TYPEC_DEAD_BATTERY_SNK               = 23,

    TYPEC_INVALID_STATE = 0xFFU,
} TypeCState_t;

/**
 * @brief Chunking layer state machine states
 * 
 * Enumeration of states for PD 3.0 extended message chunking protocol.
 * Includes receive chunk handler (RCH) and transmit chunk handler (TCH) states
 * for fragmenting/reassembling extended messages up to 260 bytes.
 */
typedef enum
{
    RCH_Wait_For_Message_From_Protocol_Layer,
    RCH_Pass_Up_Message,
    RCH_Processing_Extended_Message,
    RCH_Requesting_Chunk,
    RCH_Waiting_Chunk,
    RCH_Report_Error,
    TCH_Wait_For_Message_Request_From_Policy_Engine,
    TCH_Report_Error,
    TCH_Prepare_To_Send_Chunked_Message,
    TCH_Construct_Chunked_Message,
    TCH_Sending_Chunked_Message,
    TCH_Wait_Chunk_Request,
    TCH_Message_Received,
    TCH_Message_Sent,
} pd_chunking_layer_state_t;

#endif
