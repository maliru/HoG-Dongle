/**
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"

#include "app_util_platform.h"
#include "app_scheduler.h"
#include "nrf_drv_usbd.h"
#include "nrf_queue.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_generic.h"
#include "app_error.h"
#include "usbd_hid_mouse.h"

#define NRF_LOG_MODULE_NAME usbd_hid_mouse
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief HID generic class interface number.
 * */
#define HID_GENERIC_INTERFACE  0

/**
 * @brief HID generic class endpoint number.
 * */
#define HID_GENERIC_EPIN       NRF_DRV_USBD_EPIN1

/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    1

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 *
 * Maximum value of this define is 63 bytes. Library automatically adds
 * one byte for report ID. This means that output report size is limited
 * to 64 bytes.
 */
#define REPORT_OUT_MAXSIZE  0

/**
 * @brief HID generic class endpoints count.
 * */
#define HID_GENERIC_EP_COUNT  1

/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST()                                      \
(                                                            \
        HID_GENERIC_EPIN                                     \
)


#define USBD_INP_DEBUG_GPIO     -1                      /**< USB IN endp debug GPIO pin. */
#define USBD_SCH_DEBUG_GPIO     -1                      /**< USB scheduler debug GPIO pin. */

#if USBD_INP_DEBUG_GPIO >= 0
#define USBD_INP_DEBUG_GPIO_OFF     nrf_gpio_pin_clear(USBD_INP_DEBUG_GPIO)
#define USBD_INP_DEBUG_GPIO_ON      nrf_gpio_pin_set(USBD_INP_DEBUG_GPIO)
#define USBD_INP_DEBUG_GPIO_INIT    nrf_gpio_cfg_output(USBD_INP_DEBUG_GPIO)
#else // USBD_INP_DEBUG_GPIO
#define USBD_INP_DEBUG_GPIO_OFF
#define USBD_INP_DEBUG_GPIO_ON
#define USBD_INP_DEBUG_GPIO_INIT
#endif // USBD_INP_DEBUG_GPIO

#if USBD_SCH_DEBUG_GPIO >= 0
#define USBD_SCH_DEBUG_GPIO_OFF     nrf_gpio_pin_clear(USBD_SCH_DEBUG_GPIO)
#define USBD_SCH_DEBUG_GPIO_ON      nrf_gpio_pin_set(USBD_SCH_DEBUG_GPIO)
#define USBD_SCH_DEBUG_GPIO_INIT    nrf_gpio_cfg_output(USBD_SCH_DEBUG_GPIO)
#else // USBD_SCH_DEBUG_GPIO
#define USBD_SCH_DEBUG_GPIO_OFF
#define USBD_SCH_DEBUG_GPIO_ON
#define USBD_SCH_DEBUG_GPIO_INIT
#endif // USBD_SCH_DEBUG_GPIO


#define HID_REP_MAX_SIZE        USBD_HID_REP_MAX_SIZE   /**< The maximum size of the report */
#define HID_REP_NUM_BUFF        100                      /**< The number of report buffer */

typedef struct {
    uint8_t             len;
    uint8_t             data[HID_REP_MAX_SIZE];
} hid_rep_buff_t;

NRF_QUEUE_DEF(hid_rep_buff_t, 
              m_rep_buff, 
              HID_REP_NUM_BUFF, 
              NRF_QUEUE_MODE_NO_OVERFLOW);

/**
 * @brief User event handler.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

/**
 * @brief USB HID mouse report descriptor from BLE HID mouse example.
 */
#if 0
#define APP_USBD_HID_MOUSE_REPORT_CUSTOM {                            \
    0x05, 0x01, /* Usage Page (Generic Desktop) */                    \
    0x09, 0x02, /* Usage (Mouse)                */                    \
                                                                      \
    0xA1, 0x01, /* Collection (Application)     */                    \
                                                                      \
    /* Report ID 1: Mouse buttons + scroll/pan */                     \
    0x85, 0x02,       /* Report Id 1                               */ \
    0x09, 0x01,       /* Usage (Pointer)                           */ \
    0xA1, 0x00,       /* Collection (Physical)                     */ \
    0x95, 0x05,       /* Report Count (5)                          */ \
    0x75, 0x01,       /* Report Size (1)                           */ \
    0x05, 0x09,       /* Usage Page (Buttons)                      */ \
    0x19, 0x01,       /* Usage Minimum (01)                        */ \
    0x29, 0x05,       /* Usage Maximum (05)                        */ \
    0x15, 0x00,       /* Logical Minimum (0)                       */ \
    0x25, 0x01,       /* Logical Maximum (1)                       */ \
    0x81, 0x02,       /* Input (Data, Variable, Absolute)          */ \
    0x95, 0x01,       /* Report Count (1)                          */ \
    0x75, 0x03,       /* Report Size (3)                           */ \
    0x81, 0x01,       /* Input (Constant) for padding              */ \
    0x75, 0x08,       /* Report Size (8)                           */ \
    0x95, 0x01,       /* Report Count (1)                          */ \
    0x05, 0x01,       /* Usage Page (Generic Desktop)              */ \
    0x09, 0x38,       /* Usage (Wheel)                             */ \
    0x15, 0x81,       /* Logical Minimum (-127)                    */ \
    0x25, 0x7F,       /* Logical Maximum (127)                     */ \
    0x81, 0x06,       /* Input (Data, Variable, Relative)          */ \
    0x05, 0x0C,       /* Usage Page (Consumer)                     */ \
    0x0A, 0x38, 0x02, /* Usage (AC Pan)                            */ \
    0x95, 0x01,       /* Report Count (1)                          */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0xC0,             /* End Collection (Physical)                 */ \
                                                                      \
    /* Report ID 2: Mouse motion */                                   \
    0x85, 0x01,       /* Report Id 2                               */ \
    0x09, 0x01,       /* Usage (Pointer)                           */ \
    0xA1, 0x00,       /* Collection (Physical)                     */ \
    0x75, 0x0C,       /* Report Size (12)                          */ \
    0x95, 0x02,       /* Report Count (2)                          */ \
    0x05, 0x01,       /* Usage Page (Generic Desktop)              */ \
    0x09, 0x30,       /* Usage (X)                                 */ \
    0x09, 0x31,       /* Usage (Y)                                 */ \
    0x16, 0x01, 0xF8, /* Logical maximum (2047)                    */ \
    0x26, 0xFF, 0x07, /* Logical minimum (-2047)                   */ \
    0x81, 0x06,       /* Input (Data, Variable, Relative)          */ \
    0xC0,             /* End Collection (Physical)                 */ \
    0xC0,             /* End Collection (Application)              */ \
                                                                      \
    /* Report ID 3: Advanced buttons */                               \
    0x05, 0x0C,       /* Usage Page (Consumer)                     */ \
    0x09, 0x01,       /* Usage (Consumer Control)                  */ \
    0xA1, 0x01,       /* Collection (Application)                  */ \
    0x85, 0x03,       /* Report Id (3)                             */ \
    0x15, 0x00,       /* Logical minimum (0)                       */ \
    0x25, 0x01,       /* Logical maximum (1)                       */ \
    0x75, 0x01,       /* Report Size (1)                           */ \
    0x95, 0x01,       /* Report Count (1)                          */ \
                                                                      \
    0x09, 0xCD,       /* Usage (Play/Pause)                        */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x0A, 0x83, 0x01, /* Usage (AL Consumer Control Configuration) */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x09, 0xB5,       /* Usage (Scan Next Track)                   */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x09, 0xB6,       /* Usage (Scan Previous Track)               */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
                                                                      \
    0x09, 0xEA,       /* Usage (Volume Down)                       */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x09, 0xE9,       /* Usage (Volume Up)                         */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x0A, 0x25, 0x02, /* Usage (AC Forward)                        */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0x0A, 0x24, 0x02, /* Usage (AC Back)                           */ \
    0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field)     */ \
    0xC0              /* End Collection                            */ \
}


#else


#define APP_USBD_HID_MOUSE_REPORT_CUSTOM {\
		0x05, 0x01,     /* Usage Page (Generic Desktop) */	\
		0x09, 0x02,     /* Usage (Mouse) 								*/ 	\
		0xA1, 0x01,     /* Collection (Application) 		*/	\
                                /* Report: Mouse 																*/	\ 
		0x09, 0x01,       /* Usage (Pointer) 						*/	\
		0xA1, 0x00,       /* Collection (Physical) 			*/	\
		0x85, 0X01,																					\																			
		0x05, 0x09,																					\	
                0x19, 0x01,         /* Usage Minimum (1) 				*/	\
		0x29, 0x08,         /* Usage Maximum (8) 				*/	\
		0x15, 0x00,         /* Logical Minimum (0) 			*/	\
		0x25, 0x01,         /* Logical Maximum (1) 			*/	\
		0x75, 0x01,         /* Report Size (1) 					*/	\
		0x95, 0X08, 				/* Report Count 						*/	\
		0x81, 0x02,         /* Input (Data, Variable, Absolute) */	\
																												\
		0x05, 0x01,																					\			
		0x09, 0x38,         /* Usage (Wheel) 						*/	\
		0x15, 0x81,         /* Logical Minimum (-127) 	*/	\
		0x25, 0x7F,         /* Logical Maximum (127) 		*/	\
		0x75, 0x08,         /* Report Size (8) 					*/	\
		0x95, 0x01,         /* Report Count (1) 				*/	\
		0x81, 0x06,         /* Input (Data, Variable, Relative) */	\
																												\
		0x05, 0x01,																					\				
		0x09, 0x30,         /* Usage (X) 								*/	\
		0x09, 0x31,         /* Usage (Y) 								*/	\
		0x16, 0x01, 0xF8,   /* Logical Maximum (2047) 	*/	\
		0x26, 0xFF, 0x07,   /* Logical Minimum (-2047) 	*/	\
		0x75, 0x0C,         /* Report Size (12) 				*/	\
		0x95, 0x02,         /* Report Count (2) 				*/	\
		0x81, 0x06,         /* Input (Data, Variable, Relative) */	\
		0xC0,             	/* End Collection (Physical) 	*/	\	 
		0xC0           			/* End Collection (Application) */	\ 																									
}

#endif

/**
 * @brief Reuse HID mouse report descriptor for HID generic class
 */
APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(mouse_desc, APP_USBD_HID_MOUSE_REPORT_CUSTOM);

static const app_usbd_hid_subclass_desc_t * reps[] = {&mouse_desc};

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic,
                                HID_GENERIC_INTERFACE,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                reps,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_NONE,
                                APP_USBD_HID_PROTO_GENERIC);

/*lint -restore*/

/**
 * @brief Mark the ongoing transmission
 *
 * Marks that the report buffer is busy and cannot be used until transmission finishes
 * or invalidates (by USB reset or suspend event).
 */
static nrf_atomic_flag_t m_report_pending = 0;

static void hid_generic_reset_queue(void)
{
    nrf_atomic_flag_clear(&m_report_pending);

    ret_code_t ret = NRF_SUCCESS;
    hid_rep_buff_t rep_temp;

    /* Clear all queued reports */
    while(ret != NRF_ERROR_NOT_FOUND)
    {
        ret = nrf_queue_pop(&m_rep_buff, &rep_temp);
    }
}

static void hid_generic_process_queue(void)
{
    if (m_report_pending)
        return;

    static hid_rep_buff_t rep_buff;

    ret_code_t ret;

    ret = nrf_queue_peek(&m_rep_buff, &rep_buff);

    if (ret == NRF_SUCCESS)
    {
        /* Start the transfer */
        ret = app_usbd_hid_generic_in_report_set(
            &m_app_hid_generic,
            rep_buff.data,
            rep_buff.len);
        if (ret == NRF_SUCCESS)
        {
            nrf_atomic_flag_set(&m_report_pending);

            USBD_INP_DEBUG_GPIO_ON;
        }
    }
}

static void hid_generic_scheduled_exec(void * p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    USBD_SCH_DEBUG_GPIO_ON;

    hid_generic_process_queue();

    USBD_SCH_DEBUG_GPIO_OFF;
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            /* No output report defined for this example.*/
            ASSERT(0);
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            USBD_INP_DEBUG_GPIO_OFF;

            //NRF_LOG_INFO("Input Report Done");
            nrf_atomic_flag_clear(&m_report_pending);

            hid_rep_buff_t rep_temp;
            nrf_queue_pop(&m_rep_buff, &rep_temp);

            hid_generic_process_queue();
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_BOOT_PROTO");
            nrf_drv_usbd_setup_stall();
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_RESET:
            hid_generic_reset_queue();
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            NRF_LOG_INFO("USB SUSPPEND");
            nrf_atomic_flag_clear(&m_report_pending);
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
            break;
        case APP_USBD_EVT_DRV_RESUME:
            NRF_LOG_INFO("USB RESUME");
            nrf_atomic_flag_clear(&m_report_pending);
            break;
        case APP_USBD_EVT_STARTED:
            NRF_LOG_INFO("USB STARTED");
            hid_generic_reset_queue();
            break;
        case APP_USBD_EVT_STOPPED:
            NRF_LOG_INFO("USB STOP");
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static ret_code_t idle_handle(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    switch (report_id)
    {
        case 0:
        {
            uint8_t report[] = {0xBE, 0xEF};
            return app_usbd_hid_generic_idle_report_set(
              &m_app_hid_generic,
              report,
              sizeof(report));
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    
}

ret_code_t usbd_hid_mouse_init(void)
{
    USBD_INP_DEBUG_GPIO_OFF;
    USBD_INP_DEBUG_GPIO_INIT;
    USBD_SCH_DEBUG_GPIO_OFF;
    USBD_SCH_DEBUG_GPIO_INIT;

    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_generic;
    class_inst_generic = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic);

    ret = hid_generic_idle_handler_set(class_inst_generic, idle_handle);
    APP_ERROR_CHECK(ret);

    ret = app_usbd_class_append(class_inst_generic);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {

        NRF_LOG_INFO("USB power detect");
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
    
    return ret;
}

ret_code_t usbd_hid_mouse_in_rep_put(uint8_t *p_data, uint8_t len)
{
    VERIFY_PARAM_NOT_NULL(p_data);
    
    if (len > HID_REP_MAX_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    hid_rep_buff_t rep_temp;

    memcpy(rep_temp.data, p_data, len);
    rep_temp.len = len;

    ret_code_t ret;

    ret = nrf_queue_push(&m_rep_buff, &rep_temp);
#if 0
    // avoid racing condition with app_usbd_event_queue_process()
     if(1)
     {
          hid_generic_process_queue();
     }
#else
    // request app_scheduler to process queued report
    if (ret == NRF_SUCCESS)
    {
        ret = app_sched_event_put(NULL, 0, hid_generic_scheduled_exec);
        APP_ERROR_CHECK(ret);
    }
#endif
    //return ret;
}

void usbd_hid_mouse_routines(void)
{
    while (app_usbd_event_queue_process())
    {
        /* Nothing to do */
    }
    hid_generic_process_queue();
}
