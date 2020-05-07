/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_HIDS_C)
#include "ble_hids_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"

#define NRF_LOG_MODULE_NAME ble_hids_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define TX_BUFFER_MASK         0x0f                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;


 tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
 uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
 uint32_t      m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */
 bool          m_tx_active = false;          /**< Transmit active flag. */





/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index && !m_tx_active)
    {
        m_tx_active = true;

        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);

            NRF_LOG_INFO("SD gattc write conn_hadlle = %d gatt_params %d",m_tx_buffer[m_tx_index].conn_handle,m_tx_buffer[m_tx_index].req.read_handle);
        }

        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                          "attempted again..");
        }

    }
    else
    {
          NRF_LOG_INFO("TX Progress Failed");
    }
}

void hidc_tx_buffer(void)
{
  tx_buffer_process();
}

static void on_hvx(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    // Check if the event is on the link for this instance
    if (p_ble_hids_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        NRF_LOG_DEBUG("Received HVX on link 0x%x, not associated to this instance, ignore",
                      p_ble_evt->evt.gattc_evt.conn_handle);
    //    return;
    }

    uint8_t i;
    uint8_t n = p_ble_hids_c->peer_inp_db.inp_rep_count;

    // find the corresponding HID Report characteristic
    for (i = 0; i < n; i++)
    {
        inp_rep_char_t *p_inp_rep_char = p_ble_hids_c->peer_inp_db.inp_rep_char + i;

        if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_inp_rep_char->inp_handle)
        {
            ble_hids_c_evt_t ble_hids_c_evt;

            ble_hids_c_evt.evt_type                  = BLE_HIDS_C_EVT_INP_NOTIFICATION;
            ble_hids_c_evt.conn_handle               = p_ble_hids_c->conn_handle;
            ble_hids_c_evt.params.inp_rep.inp_rep_id = p_inp_rep_char->inp_rep_id;
            ble_hids_c_evt.params.inp_rep.p_data     = p_ble_evt->evt.gattc_evt.params.hvx.data;
            ble_hids_c_evt.params.inp_rep.len        = p_ble_evt->evt.gattc_evt.params.hvx.len;

            p_ble_hids_c->evt_handler(p_ble_hids_c, &ble_hids_c_evt);
            break;
        }
    }
}


static void on_write_rsp(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_write_rsp_t * p_response;

    // Check if the event if on the link for this instance
    if (p_ble_hids_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    p_response = &p_ble_evt->evt.gattc_evt.params.write_rsp;

    if (p_response->handle != m_tx_buffer[m_tx_index].req.write_req.gattc_params.handle)
    {
        return;
    }

    // Write request is done
    m_tx_active = false;

    if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION) ||
        (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION))
    {
        // Do nothing to reattempt write.
    }
    else
    {
        m_tx_index++;
        m_tx_index &= TX_BUFFER_MASK;
    }
    NRF_LOG_INFO("ON Write RSP");
    // Check if there is any message to be sent across to the peer and send it
    tx_buffer_process();
}


static void on_read_rsp(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;

    // Check if the event if on the link for this instance
    if (p_ble_hids_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

    if (p_response->handle != m_tx_buffer[m_tx_index].req.read_handle)
    {
        return;
    }

    // Read request is done
    m_tx_active = false;

    if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION) ||
        (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION))
    {
        // Do nothing to reattempt read.
    }
    else
    {
        if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS) &&
            (p_response->len >= 1) && !(p_response->offset))
        {
            uint8_t i;
            uint8_t n = p_ble_hids_c->peer_inp_db.inp_rep_count;

            for (i = 0; i < n; i++)
            {
                inp_rep_char_t *p_inp_rep_char = p_ble_hids_c->peer_inp_db.inp_rep_char + i;

                if (p_response->handle == p_inp_rep_char->inp_ref_handle)
                {
                    // Store HID Report ID
                    p_inp_rep_char->inp_rep_id = p_response->data[0];
                    break;
                }
            }
        }

        m_tx_index++;
        m_tx_index &= TX_BUFFER_MASK;
    }

     NRF_LOG_INFO("ON read RSP");
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


static void on_disconnected(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_hids_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_hids_c->conn_handle               = BLE_CONN_HANDLE_INVALID;
        p_ble_hids_c->peer_inp_db.inp_rep_count = 0;
    }
}


void ble_hids_on_db_disc_evt(ble_hids_c_t * p_ble_hids_c, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the HID Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
    {
        uint8_t i, n;

        ble_hids_c_evt_t evt;

        evt.evt_type    = BLE_HIDS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        n = 0;
        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_REPORT_CHAR)
            {
                if (n >= BLE_HIDS_C_MAX_INPUT_REP)
                    break;

                inp_rep_char_t *p_inp_rep_char = evt.params.peer_db.inp_rep_char + n;

                p_inp_rep_char->inp_cccd_handle = 
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_inp_rep_char->inp_ref_handle =
                    p_evt->params.discovered_db.charateristics[i].report_ref_handle;
                p_inp_rep_char->inp_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_inp_rep_char->inp_rep_id = 0;
                n++;
            }
        }
        evt.params.peer_db.inp_rep_count = n;

        NRF_LOG_DEBUG("HID Service discovered at peer.");

        p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
    }
}


uint32_t ble_hids_c_init(ble_hids_c_t * p_ble_hids_c, ble_hids_c_init_t * p_ble_hids_c_init)
{
    VERIFY_PARAM_NOT_NULL(p_ble_hids_c);
    VERIFY_PARAM_NOT_NULL(p_ble_hids_c_init);

    ble_uuid_t hids_uuid;

    hids_uuid.type = BLE_UUID_TYPE_BLE;
    hids_uuid.uuid = BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE;

    p_ble_hids_c->evt_handler               = p_ble_hids_c_init->evt_handler;
    p_ble_hids_c->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_ble_hids_c->peer_inp_db.inp_rep_count = 0;

    return ble_db_discovery_evt_register(&hids_uuid);
}

void ble_hids_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_hids_c_t * p_ble_hids_c = (ble_hids_c_t *)p_context;

    if ((p_ble_hids_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_hids_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_hids_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(p_ble_hids_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_hids_c, p_ble_evt);
            break;

        default:
            break;
    }
}


static uint32_t read_ref_id(uint16_t conn_handle, uint16_t handle_ref)
{
    tx_message_t * p_msg;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.read_handle = handle_ref;
    p_msg->conn_handle     = conn_handle;
    p_msg->type            = READ_REQ;

    NRF_LOG_INFO("read ref id")
    tx_buffer_process();
    return NRF_SUCCESS;
}


uint32_t ble_hids_c_inp_ref_read_id(ble_hids_c_t * p_ble_hids_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_hids_c);

    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
    uint8_t i;
    uint8_t n = p_ble_hids_c->peer_inp_db.inp_rep_count;

    for (i = 0; i < n; i++)
    {
        err_code = read_ref_id(p_ble_hids_c->conn_handle,
                               p_ble_hids_c->peer_inp_db.inp_rep_char[i].inp_ref_handle);
        if (err_code != NRF_SUCCESS)
            break;
    }

    return err_code;
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{


    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;


    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d ,cccd_val %d", handle_cccd,conn_handle,cccd_val);

    tx_buffer_process();
    return NRF_SUCCESS;
}


uint32_t ble_hids_c_inp_notif_enable(ble_hids_c_t * p_ble_hids_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_hids_c);

    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
    uint8_t i;
    uint8_t n = p_ble_hids_c->peer_inp_db.inp_rep_count;

    for (i = 0; i < n; i++)
    {
        err_code = cccd_configure(p_ble_hids_c->conn_handle,
                                  p_ble_hids_c->peer_inp_db.inp_rep_char[i].inp_cccd_handle,
                                  true);
        if (err_code != NRF_SUCCESS)
            break;
    }

    return err_code;
}


uint32_t ble_hids_c_handles_assign(ble_hids_c_t *       p_ble_hids_c,
                                   uint16_t             conn_handle,
                                   const inp_rep_db_t * p_peer_inp_rep_db)
{
    VERIFY_PARAM_NOT_NULL(p_ble_hids_c);

    p_ble_hids_c->conn_handle = conn_handle;
    if (p_peer_inp_rep_db != NULL)
    {
        p_ble_hids_c->peer_inp_db = *p_peer_inp_rep_db;
    }

    return NRF_SUCCESS;
}

/** @}
 *  @endcond
 */
#endif // NRF_MODULE_ENABLED(BLE_HIDS_C)
