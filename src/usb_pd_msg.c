/**
 * @file usb_pd_msg.c
 * @brief USB PD Message Handling Implementation
 * 
 * @details Implements USB PD message transmission, reception, and protocol handling.
 *          Manages message ID sequencing, GoodCRC handling, and message buffering.
 * 
 *          Key responsibilities:
 *          - Message ID management (0-7 counter per SOP type)
 *          - Message transmission with retry handling
 *          - Message reception with duplicate filtering
 *          - Extended message chunking (PD 3.0)
 *          - Structured VDM transmission
 *          - Hard reset transmission/reception
 * 
 *          Message ID protocol:
 *          - Separate counters for SOP/SOP'/SOP''
 *          - Increment on successful transmission
 *          - Reset on hard reset or soft reset
 *          - Duplicate detection using received message ID cache
 * 
 *          Message flow:
 *          TX: PD_MsgSend() → PHY transmission → PD_MsgSendDone() callback
 *          RX: PHY reception → PD_MsgReceived() → Policy engine processing
 * 
 *          Header masking for cable communication:
 *          - SOP: All bits valid (0xFFFFFFFF)
 *          - SOP' cable: Mask certain bits (0xFFDFFFFF)
 *          - SOP' non-cable: Different mask (0xFEDFFFFF)
 * 
 * @copyright Copyright 2015 - 2017 NXP. All rights reserved.
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_MSG_HEADER_MASK_FOR_SOP            (0xFFFFFFFFu)
#define PD_MSG_HEADER_MASK_FOR_SOPP_CABLE     (0xFFDFFFFFu)
#define PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE (0xFEDFFFFFu)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void PD_MsgResetMsgId(pd_instance_t *pdInstance, start_of_packet_t sop);
static void PD_MsgResetAllMsgId(pd_instance_t *pdInstance);
static inline void PD_MsgIncrMsgId(pd_instance_t *pdInstance, start_of_packet_t sop);
static pd_status_t PD_MsgPrevSendCheck(pd_instance_t *pdInstance);
static void PD_MsgCopy(void *dst, void *src, uint32_t size);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_MsgResetMsgId(pd_instance_t *pdInstance, start_of_packet_t sop)
{
    pdInstance->msgId[sop]              = 0;
    pdInstance->rcvMsgId[sop]           = 0;
    pdInstance->firstMsgAfterReset[sop] = 1;
}

static void PD_MsgResetAllMsgId(pd_instance_t *pdInstance)
{
    (void)memset(pdInstance->msgId, 0, sizeof(pdInstance->msgId));
    (void)memset(pdInstance->rcvMsgId, 0, sizeof(pdInstance->rcvMsgId));
    (void)memset(pdInstance->firstMsgAfterReset, 1, sizeof(pdInstance->firstMsgAfterReset));
}

static inline void PD_MsgIncrMsgId(pd_instance_t *pdInstance, start_of_packet_t sop)
{
    pdInstance->msgId[sop] = (uint8_t)(pdInstance->msgId[sop] + 1U) & (uint8_t)0x07U;
}

/**
 * @brief Reset message handling subsystem
 * 
 * Resets all message IDs, PHY message function, and message state flags.
 * Called during initialization and hard reset recovery.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReset(pd_instance_t *pdInstance)
{
    PD_MsgResetAllMsgId(pdInstance);
    (void)PD_PhyControl(pdInstance, PD_PHY_RESET_MSG_FUNCTION, NULL);
    pdInstance->hardResetReceived = 0;
    pdInstance->sendingState      = 0;
    pdInstance->receiveState      = 0;
    pdInstance->pendingMsg        = 0;
    pdInstance->receivedData      = NULL;
}

/**
 * @brief Notify message send completion
 * 
 * Called by PHY layer when message transmission completes (success or failure).
 * Updates message ID on successful transmission, sets event flag for policy engine.
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] result Send result (kStatus_PD_Success or error)
 */
void PD_MsgSendDone(pd_instance_t *pdInstance, pd_status_t result)
{
    if (pdInstance->sendingState == 1U)
    {
        PD_MsgIncrMsgId(pdInstance, pdInstance->sendingSop);
        pdInstance->sendingState  = 2;
        pdInstance->sendingResult = result;
        PD_StackSetEvent(pdInstance, PD_TASK_EVENT_SEND_DONE);
    }
}

static pd_status_t PD_MsgPrevSendCheck(pd_instance_t *pdInstance)
{
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    if ((pdInstance->hardResetReceived != 0U) || (pdInstance->sendingState == 1U))
    {
        OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }

    pdInstance->sendingState = 1;
    OSA_EXIT_CRITICAL();
    return kStatus_PD_Success;
}

static void PD_MsgCopy(void *dst, void *src, uint32_t size)
{
    if (dst == src)
    {
        return;
    }
    else
    {
        (void)memcpy(dst, src, size);
    }
}

/**
 * @brief Send PD message (control, data, or extended)
 * 
 * Constructs and sends USB PD message with proper header formatting:
 * - Sets message type, data role, power role, spec revision
 * - Assigns message ID and increments on success
 * - Handles SOP/SOP'/SOP'' header masking
 * - Copies data objects to send buffer
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] sop Start of packet type (SOP/SOP'/SOP'')
 * @param[in] msgType Message type (control/data/extended)
 * @param[in] dataLength Number of data objects (0 for control messages)
 * @param[in] dataBuffer Pointer to data objects (can be NULL for control)
 * @return kStatus_PD_Success on success, error code otherwise
 */
pd_status_t PD_MsgSend(
    pd_instance_t *pdInstance, start_of_packet_t sop, message_type_t msgType, uint32_t dataLength, uint8_t *dataBuffer)
{
    pd_msg_header_t msgHeader;
    if (PD_MsgPrevSendCheck(pdInstance) != kStatus_PD_Success)
    {
        return kStatus_PD_Error;
    }

    if (msgType == kPD_MsgSoftReset)
    {
        PD_MsgResetMsgId(pdInstance, sop);
    }

    /* Set message header */
    pdInstance->sendingSop                       = sop;
    msgHeader.msgHeaderVal                       = 0;
    msgHeader.bitFields.portPowerRoleOrCablePlug = (uint16_t)pdInstance->curPowerRole;
    msgHeader.bitFields.portDataRole             = (uint16_t)pdInstance->curDataRole;
    msgHeader.bitFields.specRevision             = pdInstance->revision;
    if (dataLength < 2U)
    {
        pdInstance->sendingResult = kStatus_PD_Error;
        return pdInstance->sendingResult;
    }
    msgHeader.bitFields.NumOfDataObjs            = (uint16_t)((dataLength - 2U) >> 2U); /* control, data, chunked */
    msgHeader.bitFields.messageID                = (uint16_t)pdInstance->msgId[sop];
    msgHeader.bitFields.messageType              = (uint16_t)msgType & PD_MSG_TYPE_VALUE_MASK;
    if (((uint8_t)msgType & PD_MSG_EXT_TYPE_MASK) == PD_MSG_EXT_TYPE_MASK)
    {
        msgHeader.bitFields.extended = 1;
        if (pdInstance->unchunkedFeature != 0U)
        {
            msgHeader.bitFields.NumOfDataObjs = 0; /* unchunked */
        }
    }
    else
    {
        msgHeader.bitFields.extended = 0;
    }
    pdInstance->sendingData[0] = (uint32_t)msgHeader.msgHeaderVal << 16U;

    if ((sop != kPD_MsgSOP) && (pdInstance->pdConfig->deviceType != (uint8_t)kDeviceType_Cable))
    {
        /* Clear bit5(Port Data Role) and bit8(Cable Plug. Message originated from a DFP or UFP) */
        pdInstance->sendingData[0] &= PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE;
    }
    /* Copy message data */
    PD_MsgCopy(&pdInstance->sendingData[1], dataBuffer, dataLength - 2U);

    PD_MsgReceive(pdInstance);
    pdInstance->sendingResult = kStatus_PD_Error;
    if (pdInstance->phyInterface->pdPhySend(pdInstance, (uint8_t)sop, (uint8_t *)&pdInstance->sendingData[0],
                                            dataLength) == kStatus_PD_Success)
    {
        /* Wait for the result to be sent. */
        (void)PD_PsmTimerWait(pdInstance, timrMsgSendWaitResultTimer, T_WAIT_SEND_RESULT, PD_TASK_EVENT_SEND_DONE);
        (void)OSA_EventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
    }

    pdInstance->sendingState = 0;

    return pdInstance->sendingResult;
}

/*!
 * @brief Send extended message.
 *
 * This function is used to send extended message.
 *
 * @param pdInstance      The pd handle. It equals the value returned from PD_InstanceInit.
 * @param sop             The sending sop. Please refer to the enumeration start_of_packet_t.
 * @param extMsgType      Message type. Please refer to the enumeration message_type_t.
 * @param dataLength      The length of buffer. It is from Byte0 to ByteN.
 * @param dataBuffer      The sending buffer. The pointer of dataBuffer points to Byte0.
 *                        The format of buffer. 0-NULL, 1-I2C_WRITE_BYTE_COUNT, 2-MsgHeaderLowByte,
 *                        3-MsgHeaderHighByte, 4-ExtHeaderLowByte, 5-ExtHeaderHighByte, 6-Byte0, 7-Byte1, ..., N-ByteN.
 * @param dataSize        The field of extended message header. This value should be 0 when requestChunk equals to 1;
 * @param requestChunk    The field of extended message header. This value should be 0 when sending unckunked message.
 * @param chunkNumber     The field of extended message header. This value should be 0 when sending unckunked message.
 *
 * @return kStatus_PD_Error or kStatus_PD_Success.
 */
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
pd_status_t PD_MsgSendExtendedMsg(pd_instance_t *pdInstance,
                                  start_of_packet_t sop,
                                  message_type_t extMsgType,
                                  uint32_t dataLength,
                                  uint8_t *dataBuffer,
                                  uint16_t dataSize,
                                  uint8_t requestChunk,
                                  uint8_t chunkNumber)
{
    uint32_t index;
    uint8_t *dst;
    uint32_t sendlength;
    pd_extended_msg_header_t extMsgHeader;

    /* set extended msg header */
    extMsgHeader.extendedMsgHeaderVal = 0;
    if (pdInstance->unchunkedFeature != 0U)
    {
        extMsgHeader.bitFields.chunked = 0;
    }
    else
    {
        extMsgHeader.bitFields.chunked = 1;
    }
    extMsgHeader.bitFields.dataSize     = dataSize;
    extMsgHeader.bitFields.requestChunk = requestChunk;
    extMsgHeader.bitFields.chunkNumber  = chunkNumber;
    pdInstance->sendingData[1]          = extMsgHeader.extendedMsgHeaderVal;

    /* copy msg data */
    dst = (uint8_t *)pdInstance->sendingData + 6;
    PD_MsgCopy(dst, dataBuffer, dataLength);

    /* pad zero */
    if (pdInstance->unchunkedFeature == 0U)
    {
        sendlength = ((dataLength + 2U + 3U) >> 2U);                 /* add extended msg header */
        sendlength = (sendlength << 2U) + 2U;                        /* add msg header */
        for (index = dataLength; index < (sendlength - 4U); ++index) /* subtract extended msg header */
        {
            dst[index] = 0U;
        }
    }
    else
    {
        sendlength = dataLength + 4U; /* add msg header, extended msg header */
    }

    return PD_MsgSend(pdInstance, sop, extMsgType, sendlength, (uint8_t *)&pdInstance->sendingData[1]);
}
#endif

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) ||        \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
pd_status_t PD_MsgSendStructuredVDM(pd_instance_t *pdInstance,
                                    start_of_packet_t sop,
                                    pd_structured_vdm_header_t reponseVdmHeader,
                                    uint8_t count,
                                    uint32_t *vdos)
{
    /* set structured vdm header */
    pdInstance->sendingData[1] = reponseVdmHeader.structuredVdmHeaderVal;
    /* copy msg data */
    PD_MsgCopy(&pdInstance->sendingData[2], vdos, (uint32_t)count * 4U);

    return PD_MsgSend(pdInstance, sop, kPD_MsgVendorDefined, ((uint32_t)count + 1U) * 4U + 2U,
                      (uint8_t *)&pdInstance->sendingData[1]);
}
#endif

/**
 * @brief Handle hard reset reception
 * 
 * Called by PHY when hard reset received on CC line. Resets all message IDs
 * and posts hard reset event to policy engine.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReceivedHardReset(pd_instance_t *pdInstance)
{
    pdInstance->hardResetReceived = 1;
    PD_MsgResetAllMsgId(pdInstance);
    PD_StackSetEvent(pdInstance, PD_TASK_EVENT_RECEIVED_HARD_RESET);
}

/**
 * @brief Transmit hard reset or cable reset
 * 
 * Initiates hard reset (clears message state) or cable reset (SOP' targeted).
 * Resets all message IDs and triggers PHY to send reset signaling.
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] hardResetOrCableReset 0 for hard reset, 1 for cable reset
 */
void PD_MsgSendHardOrCableReset(pd_instance_t *pdInstance, uint8_t hardResetOrCableReset)
{
    /* Buffer will now be used for TX, Hard Reset is allowed to be sent while the TCPC is still processing a previous */
    /* transmission */
    pdInstance->sendingState = 1;

    if (hardResetOrCableReset == 0U)
    {
        (void)PD_PhyControl(pdInstance, PD_PHY_SEND_HARD_RESET, NULL);
    }
    else
    {
        (void)PD_PhyControl(pdInstance, PD_PHY_SEND_CABLE_RESET, NULL);
    }

    (void)PD_PsmTimerWait(pdInstance, tMsgHardResetCompleteTimer, T_HARD_RESET_COMPLETE, PD_TASK_EVENT_SEND_DONE);

    (void)OSA_EventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
    pdInstance->sendingState = 0;
    if (hardResetOrCableReset == 0U) /* Hard Reset */
    {
        PD_MsgResetAllMsgId(pdInstance);
    }
    else /* Cable Reset */
    {
        /* Reset all sop' sop'' msgid  after sending cable reset */
        PD_MsgResetMsgId(pdInstance, kPD_MsgSOPp);
        PD_MsgResetMsgId(pdInstance, kPD_MsgSOPpp);
        PD_MsgResetMsgId(pdInstance, kPD_MsgSOPDbg);
        PD_MsgResetMsgId(pdInstance, kPD_MsgSOPpDbg);
    }
}

/**
 * @brief Process received message from PHY
 * 
 * Handles message reception from PHY layer:
 * - Validates message header and SOP type
 * - Checks message ID for duplicates
 * - Caches received data for policy engine
 * - Posts message received event
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] rxResult RX result structure from PHY
 */
void PD_MsgReceived(pd_instance_t *pdInstance, pd_phy_rx_result_t *rxResult)
{
    pd_msg_header_t msgHeader;

    if (pdInstance->receiveState == 1U)
    {
        msgHeader.msgHeaderVal =
            USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)pdInstance->receivingDataBuffer + 2U));

        if ((msgHeader.bitFields.messageType == (uint8_t)kPD_MsgSoftReset) && (msgHeader.bitFields.extended == 0U) &&
            (msgHeader.bitFields.NumOfDataObjs == 0U))
        {
            /* SoftReset Message always has a MessageID value of zero */
            PD_MsgResetMsgId(pdInstance, ((rxResult->rxSop == kPD_MsgSOP) ? kPD_MsgSOP : kPD_MsgSOPp));
        }
        else if ((msgHeader.bitFields.messageID != pdInstance->rcvMsgId[rxResult->rxSop]) ||
                 (pdInstance->firstMsgAfterReset[rxResult->rxSop] != 0U))
        {
            /* Drop the duplicate messages */
            /* 1.When the first good packet is received after a reset, the receiver shall store a copy of the received
             */
            /* MessageID value */
            /* 2.If MessageID value in the received Message is different than the stored value, the receiver shall
               return a GoodCRC */
            /* Message with the new MessageID value, store a copy of the new MessageID value and process the Message. */
            pdInstance->firstMsgAfterReset[rxResult->rxSop] = 0;
            pdInstance->rcvMsgId[rxResult->rxSop]           = (uint8_t)msgHeader.bitFields.messageID;
        }
        else
        {
            /* For subsequent Messages, if MessageID value in a received Message is the same as the stored value, */
            /* the receiver shall return a GoodCRC Message with that MessageID value and drop the Message */
            /* (this is a retry of an already received Message). */
            pdInstance->receiveState = 0;
            PD_MsgReceive(pdInstance);
            return;
        }

        pdInstance->receiveState   = 2;
        pdInstance->receivedSop    = rxResult->rxSop;
        pdInstance->receivedLength = rxResult->rxLength;
        pdInstance->receiveResult  = rxResult->rxResultStatus;
        PD_StackSetEvent(pdInstance, PD_TASK_EVENT_PD_MSG);
    }
}

/**
 * @brief Stop message reception
 * 
 * Disables message reception in PHY. Used during disconnection or
 * when transitioning to states that don't accept messages.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgStopReceive(pd_instance_t *pdInstance)
{
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    pdInstance->enableReceive = 0;
    if (pdInstance->receiveState == 1U)
    {
        pdInstance->receiveState = 0;
        OSA_EXIT_CRITICAL();
    }
    else
    {
        OSA_EXIT_CRITICAL();
    }
}

/**
 * @brief Start message reception
 * 
 * Enables message reception in PHY. Called when entering states that
 * should monitor for incoming PD messages.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgStartReceive(pd_instance_t *pdInstance)
{
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    pdInstance->enableReceive = 1;
    OSA_EXIT_CRITICAL();
    PD_MsgReceive(pdInstance);
}

/**
 * @brief Check for and initiate message reception
 * 
 * Initiates PHY-level reception if receive state is idle and a buffer
 * is available. Called periodically to monitor for incoming messages.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgReceive(pd_instance_t *pdInstance)
{
    if (pdInstance->enableReceive == 0U)
    {
        return;
    }

    if (pdInstance->receiveState == 0U)
    {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        if ((pdInstance->receivingDataBuffer == NULL) ||
            ((pdInstance->unchunkedFeature != 0U) &&
             (pdInstance->receivingDataBuffer == (uint32_t *)&pdInstance->receivingChunkedData[0])))
#else
        if (pdInstance->receivingDataBuffer == NULL)
#endif
        {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            if (pdInstance->unchunkedFeature != 0U)
            {
                pdInstance->receivingDataBuffer = (uint32_t *)&pdInstance->receivingData[0];
            }
            else
            {
                pdInstance->receivingDataBuffer = (uint32_t *)&pdInstance->receivingChunkedData[0];
            }
#else
            pdInstance->receivingDataBuffer = (uint32_t *)&pdInstance->receivingData[0];
#endif
        }
        pdInstance->receiveState = 1;
        (void)pdInstance->phyInterface->pdPhyReceive(pdInstance, pdInstance->pendingSOP,
                                                     (uint8_t *)&pdInstance->receivingDataBuffer[0]);
    }
}

/**
 * @brief Get message reception result
 * 
 * Checks if message reception has completed and processes the received
 * message. Handles extended message chunking if enabled. Returns success
 * status and prepares next reception.
 * 
 * @param[in] pdInstance PD instance pointer
 * @return uint8_t 1 if message received successfully, 0 otherwise
 */
uint8_t PD_MsgGetReceiveResult(pd_instance_t *pdInstance)
{
    uint8_t retVal = 0;
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t receiveLength;
    uint8_t *destBuffer;
    uint8_t *sourceBuffer;
    uint8_t needCopy = 0;
    pd_msg_header_t msgHeader;
    pd_extended_msg_header_t extHeader;
#endif

    if (pdInstance->receiveState == 2U)
    {
        pdInstance->receiveState = 0;
        if (pdInstance->receiveResult == kStatus_PD_Success)
        {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            sourceBuffer = (uint8_t *)pdInstance->receivingDataBuffer;
            sourceBuffer += 2U;
            msgHeader.msgHeaderVal = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(sourceBuffer);
            if (msgHeader.bitFields.extended != 0U)
            {
                sourceBuffer += 2U;
                extHeader.extendedMsgHeaderVal = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(sourceBuffer);

                /* chunked ext msg need copy, because it may be larger than 28 bytes and are chunked */
                if (pdInstance->receivingDataBuffer == (uint32_t *)&pdInstance->receivingChunkedData[0])
                {
                    needCopy = 1;

                    /* copy msg header */
                    pdInstance->receivingData[0] =
                        USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)pdInstance->receivingDataBuffer));
                    /* skip count, frame type, msg header */
                    destBuffer   = (uint8_t *)&pdInstance->receivingData[0] + 4U;
                    sourceBuffer = (uint8_t *)pdInstance->receivingDataBuffer + 4U;
                    /* copy ext header */
                    destBuffer[0] = sourceBuffer[0];
                    destBuffer[1] = sourceBuffer[1];
                    /* skip ext header */
                    destBuffer += 2;
                    sourceBuffer += 2;
                }
                else
                {
                    needCopy = 0;
                }

                receiveLength = pdInstance->receivedLength; /* receivelength is from msgHeader to ByteN */

                if (extHeader.bitFields.chunked == 0U)
                {
                    /* unchunked ext msg */
                    if (receiveLength > 264U)
                    {
                        receiveLength = 264U;
                    }
                }
                else
                {
                    /* chunked ext msg */
                    if (receiveLength > 30U)
                    {
                        receiveLength = 30U;
                    }

                    if (needCopy != 0U)
                    {
                        destBuffer += (extHeader.bitFields.chunkNumber * 26U);
                    }
                }
            }
            else
#endif
            {
                /* control or data msg */
            }

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            if (needCopy != 0U)
            {
                /* reduce the normal header and ext header */
                /* copy the remaining bytes except msgHeader and extHeader */
                PD_MsgCopy(destBuffer, sourceBuffer, receiveLength - 4U);
                pdInstance->receivedData = (uint32_t *)&(pdInstance->receivingData[0]);
            }
            else
#endif
            {
                pdInstance->receivedData = pdInstance->receivingDataBuffer;
            }
        }

        retVal = (pdInstance->receiveResult == kStatus_PD_Success) ? 1U : 0U;
    }

    return retVal;
}

/**
 * @brief Check if message reception is pending
 * 
 * Returns whether a receive operation has completed and is waiting
 * to be processed by policy engine.
 * 
 * @param[in] pdInstance PD instance pointer
 * @return uint8_t 1 if reception pending, 0 otherwise
 */
uint8_t PD_MsgRecvPending(pd_instance_t *pdInstance)
{
    return (pdInstance->receiveState == 2U) ? 1U : 0U;
}

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
/**
 * @brief Start AMS as source (PD 3.0 collision avoidance)
 * 
 * Implements PD 3.0 AMS collision avoidance by setting Rp to 1.5A and
 * waiting tSinkTx before starting source-initiated AMS. Only active
 * when PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE is defined.
 * 
 * @param[in] pdInstance PD instance pointer
 * @return uint8_t 1 to start AMS, 0 to wait
 */
/* return 0 wait, 1 start */
uint8_t PD_MsgSrcStartCommand(pd_instance_t *pdInstance)
{
    if (pdInstance->revision >= PD_SPEC_REVISION_30)
    {
        if (pdInstance->commandSrcOwner == 0U)
        {
            typec_current_val_t rpVal = kCurrent_1A5;
            uint16_t timrTime         = T_SINK_TX;

            pdInstance->commandSrcOwner = 1;
            (void)PD_PhyControl(pdInstance, PD_PHY_SRC_SET_TYPEC_CURRENT_CAP, &rpVal);

            do
            {
                timrTime =
                    PD_PsmTimerWait(pdInstance, tSinkTxTimer, timrTime,
                                    ((uint32_t)PD_TASK_EVENT_RECEIVED_HARD_RESET | (uint32_t)PD_TASK_EVENT_PD_MSG));

                if (timrTime != 0U)
                {
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    uint32_t eventSet = 0U;
                    (void)OSA_EventGet(pdInstance->taskEventHandle, PD_TASK_EVENT_ALL, &eventSet);

                    if (((PD_TASK_EVENT_PD_MSG & eventSet) != 0U) && (pdInstance->receivedSop != kPD_MsgSOP))
                    {
                        /* discard sop' sop'' message */
                        (void)OSA_EventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
                        pdInstance->receiveState = 0U;
                        PD_MsgReceive(pdInstance);
                        continue;
                    }
#endif
                    return 0;
                }
                else
                {
                    break;
                }
            } while (true);
        }
    }

    return 1;
}

/**
 * @brief End AMS as source (PD 3.0 collision avoidance)
 * 
 * Restores Rp to 3.0A after completing source-initiated AMS, allowing
 * sink to initiate messages again. Only active when collision avoidance
 * is enabled.
 * 
 * @param[in] pdInstance PD instance pointer
 */
void PD_MsgSrcEndCommand(pd_instance_t *pdInstance)
{
    if ((pdInstance->commandSrcOwner != 0U) && (pdInstance->revision >= PD_SPEC_REVISION_30))
    {
        pdInstance->commandSrcOwner = 0U;
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            typec_current_val_t rpVal = kCurrent_3A;
            (void)PD_PhyControl(pdInstance, PD_PHY_SRC_SET_TYPEC_CURRENT_CAP, &rpVal);
        }
    }
}

/**
 * @brief Check if sink can start command (PD 3.0 collision avoidance)
 * 
 * Implements PD 3.0 sink-side AMS collision avoidance by checking if
 * source's Rp is at 3.0A before initiating sink-originated AMS. Returns
 * 0 if Rp is 1.5A (source owns AMS), 1 if safe to proceed.
 * 
 * @param[in] pdInstance PD instance pointer
 * @param[in] command Command to be executed
 * @return uint8_t 1 if allowed to start, 0 to wait
 */
uint8_t PD_MsgSnkCheckStartCommand(pd_instance_t *pdInstance, pd_command_t command)
{
    if (pdInstance->revision >= PD_SPEC_REVISION_30)
    {
        /* soft_reset for protocol error and hard_reset don't care Rp */
        if ((command != PD_DPM_CONTROL_HARD_RESET) && (command != PD_DPM_CONTROL_SOFT_RESET))
        {
            typec_current_val_t rpVal = kCurrent_3A;

            (void)PD_PhyControl(pdInstance, PD_PHY_GET_TYPEC_CURRENT_CAP, &rpVal);
            if (rpVal != kCurrent_3A)
            {
                return 0;
            }
        }
    }

    return 1;
}
#endif /* PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE */
