/******************************************************************************
* File Name: main.c
*
* Description:This is the source code for the AnyCloud: BLE CTS Client Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "app_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "cyhal.h"
#include "stdio.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "cyabs_rtos.h"
#include "stdlib.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* LED pin assignment for advertising event */
#define ADV_LED_GPIO                    CYBSP_USER_LED1
/* PWM frequency of LED's in Hz when blinking */
#define ADV_LED_PWM_FREQUENCY           (1)
/* CTS Service UUID - used for service discovery */
#define CTS_SERVICE_UUID                (0x1805)
/* CTS Service UUID length */
#define CTS_SERVICE_UUID_LEN            (2)
/* Current Time characteristic descriptor length - used to subscribe for notification */
#define CCCD_LENGTH                     CTS_SERVICE_UUID_LEN
/* Value used to derive handle of CCCD of current time characteristic */
#define GATT_CCCD_HANDLE                (3)

/*******************************************************************************
*        Enumerations and Constants
*******************************************************************************/
/* PWM Duty Cycle of LED's for different states */
enum
{
    LED_ON_DUTY_CYCLE = 0,
    LED_BLINKING_DUTY_CYCLE= 50,
    LED_OFF_DUTY_CYCLE = 100
} led_duty_cycles;

/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

/* This enumeration represents the postions of bitfield in 'adjust reason' field
   of notification */
typedef enum
{
    MANUAL_TIME_UPDATE = 0x01,
    EXTERNAL_REFERENCE_TIME_UPDATE = 0x02,
    CHANGE_OF_TIME_ZONE = 0x04,
    CHANGE_OF_DST = 0x08,
}adjust_reason_bits_t;

/* Array to hold strings for names of days of the week */
const char* day_of_week_str[]=
{
    "UNKNOWN",
    "MONDAY",
    "TUESDAY",
    "WEDNESDAY",
    "THURSDAY",
    "FRIDAY",
    "SATURDAY",
    "SUNDAY"
};

/******************************************************************************
 * For application trace printing, you can use either OS/platform-specific
 * trace function (for example, printf(...)) directly,
 * or the macro APP_TRACE_DEBUG which is the common logging interface defined
 * by the header file cybt_platform_trace.h.
 *
 * If APP_TRACE_DEBUG is selected, please check cybt_platform_trace.h and see
 * whether CYBT_PLATFORM_TRACE_ENABLE is defined and the default level value
 * INITIAL_TRACE_LEVEL_APP.
 *
 * WICED_BT_TRACE was used in WICED SDK, now you can re-define it
 * or replace it with above.
 ******************************************************************************/
#define WICED_BT_TRACE      APP_TRACE_DEBUG

/*******************************************************************************
*        Structures
*******************************************************************************/
typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  day_of_week;
    uint8_t  fractions_256;
    uint8_t  adjust_reason;
}current_time_data_t;

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
static cyhal_pwm_t                 adv_led_pwm;
static uint16_t                    bt_connection_id = 0;
static app_bt_adv_conn_mode_t      app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
static current_time_data_t         time_date_notif;
static uint16_t                    cts_service_handle = 0;
static bool                        cts_service_found = false;

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void                   adv_led_update                 (void);
static void                   ble_app_init                   (void);
static void                   ble_app_set_advertisement_data (void);
static wiced_bt_gatt_status_t enable_gatt_notification       (void);
static void                   print_notification_data        (wiced_bt_gatt_data_t notif_data);
const  char*                  get_day_of_week                (uint8_t day);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t ble_app_connect_callback       (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t ble_app_gatt_event_handler     (wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data);

/* Callback function for Bluetooth stack management events */
static wiced_bt_dev_status_t  app_bt_management_callback     (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main()
{
    cy_rslt_t rslt;
    wiced_result_t result;

    /* Initialize the board support package */
    rslt = cybsp_init();
    if (CY_RSLT_SUCCESS != rslt)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    cybt_platform_config_init(&bt_platform_cfg_settings);

    printf("**********************AnyCloud Example*************************\n");
    printf("**** Current Time Service (CTS) - Client Application Start ****\n\n");

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!! \n");
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* The application should never reach here */
    CY_ASSERT(0) ;
}

/**************************************************************************************************
* Function Name: app_bt_management_callback()
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */

            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                ble_app_init();
            }

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                printf("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(bt_connection_id == 0)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                printf("Advertisement started\n");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }

            /* Update Advertisement LED to reflect the updated state */
            adv_led_update();
            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
            break;
    }

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_init()
***************************************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called from the BT
*   management callback once the BLE stack enabled event (BTM_ENABLED_EVT) is triggered
*   This function is executed in the BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
*************************************************************************************************/
static void ble_app_init(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_result_t result = WICED_BT_SUCCESS;

    printf("\n***********************************************\n");
    printf("**Discover device with \"BLE CTS Client\" name*\n");
    printf("***********************************************\n\n");

    /* Initialize the PWM used for Advertising LED */
    rslt = cyhal_pwm_init(&adv_led_pwm, ADV_LED_GPIO , NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS !=  rslt)
    {
        printf("Advertisement LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set Advertisement Data */
    ble_app_set_advertisement_data();

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(ble_app_gatt_event_handler);
    printf("GATT event Handler registration status: %s \n", get_bt_gatt_status_name(status));

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n", get_bt_gatt_status_name(status));

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    if(WICED_BT_SUCCESS != result)
    {
        printf("Advertisement cannot start because of error: %d \n",result);
        CY_ASSERT(0);
    }
}

/**************************************************************************************************
* Function Name: ble_app_gatt_event_handler()
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : BLE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to BLE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_gatt_event_handler(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    /* Call the appropriate callback function based on the GATT event type, and
     * pass the relevant event parameters to the callback function */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            status = ble_app_connect_callback( &p_event_data->connection_status );
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            /* Check if CTS service UUID is present*/
            if (CTS_SERVICE_UUID == p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid16)
            {
                /* Copy the CTS service UUID*/
                cts_service_handle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
                cts_service_found = true;
            }
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            if(cts_service_found)
            {
                printf("CTS service found\n");
                if(WICED_BT_GATT_SUCCESS != enable_gatt_notification())
                {
                    printf("Enable notification failed!\n");
                }
            }
            else
            {
                printf("CTS service not found\n");
            }
            break;

        case GATT_OPERATION_CPLT_EVT:
            switch (p_event_data->operation_complete.op)
            {
                case GATTC_OPTYPE_WRITE:
                    /* Check if GATT operation of enable/disable notification is success. */
                    if ((p_event_data->operation_complete.response_data.handle == (cts_service_handle + GATT_CCCD_HANDLE))
                        && (WICED_BT_GATT_SUCCESS == p_event_data->operation_complete.status))
                    {
                        printf("Notifications enabled\n");
                    }
                    else
                    {
                        printf("CCCD update failed. Error: %d\n", p_event_data->operation_complete.status);
                    }
                    break;

                case GATTC_OPTYPE_NOTIFICATION:
                    /* Function call to print the time and date notifcation */
                    print_notification_data(p_event_data->operation_complete.response_data.att_value);
                    break;
            }
            break;

        default:
            status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_set_advertisement_data()
***************************************************************************************************
* Summary:
*   This function configures the advertisement packet data
*
**************************************************************************************************/
static void ble_app_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t adv_appearance[] = { BIT16_TO_8( APPEARANCE_GENERIC_CLOCK ) };
    uint8_t num_elem = 0;

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = app_gap_device_name_len;
    adv_elem[num_elem].p_data = app_gap_device_name;
    num_elem++;

    /* Advertisement Element for Appearance */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = sizeof(adv_appearance);
    adv_elem[num_elem].p_data = adv_appearance;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/**************************************************************************************************
* Function Name: ble_app_connect_callback()
***************************************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_discovery_param_t gatt_discovery_setup = {0};

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            printf("Connected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d'\n", p_conn_status->conn_id );

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;

            /* Send GATT service discovery request */
            gatt_discovery_setup.s_handle = 1;
            gatt_discovery_setup.e_handle = 0xFFFF;
            gatt_discovery_setup.uuid.len = CTS_SERVICE_UUID_LEN;
            gatt_discovery_setup.uuid.uu.uuid16 = CTS_SERVICE_UUID;

            if(WICED_BT_GATT_SUCCESS != (status = wiced_bt_gatt_send_discover(
                                                 bt_connection_id,
                                                 GATT_DISCOVER_SERVICES_BY_UUID,
                                                 &gatt_discovery_setup)))
            {
                printf("GATT Discovery request failed. Error code: %d, Conn id: %d\n", status, bt_connection_id);
            }

        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;

            /* Reset the flags */
            cts_service_found = false;
            /* Restart the advertisements */
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            if(WICED_BT_SUCCESS != result)
            {
                printf("Advertisement cannot start because of error: %d \n",result);
                CY_ASSERT(0);
            }

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;

        }

        /* Update Advertisement LED to reflect the updated state */
        adv_led_update();

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: adv_led_update()
********************************************************************************
*
* Summary:
*   This function updates the advertising LED state based on BLE advertising/
*   connection state
*
*******************************************************************************/
static void adv_led_update(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    cy_rslt_t start_rslt = CY_RSLT_SUCCESS;
    /* Stop the advertising led pwm */
    rslt = cyhal_pwm_stop(&adv_led_pwm);
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Failed to stop PWM !!\n");
    }

    /* Update LED state based on BLE advertising/connection state.
     * LED OFF for no advertisement/connection, LED blinking for advertisement
     * state, and LED ON for connected state  */
    switch(app_bt_adv_conn_state)
    {
        case APP_BT_ADV_OFF_CONN_OFF:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_ON_CONN_OFF:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_BLINKING_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_OFF_CONN_ON:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_ON_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        default:
            /* LED OFF for unexpected states */
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;
    }
    /* Check if update to PWM parameters is successful*/
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Failed to set duty cycle parameters!!\n");
    }

    /* Start the advertising led pwm */
    start_rslt = cyhal_pwm_start(&adv_led_pwm);

    /* Check if PWM started successfully */
    if (CY_RSLT_SUCCESS != start_rslt)
    {
        printf("Failed to start PWM !!\n");
    }
}

/*****************************************************************************************
* Function Name: enable_gatt_notification()
******************************************************************************************
* Summary:
*   Enable GATT notification from the server.
*
* Parameters:
*   None
*
* Return:
*   wiced_bt_gatt_status_t  : Status code from wiced_bt_gatt_status_e.
*
****************************************************************************************/
static wiced_bt_gatt_status_t enable_gatt_notification(void)
{
    wiced_bt_gatt_value_t      *p_write = NULL;
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;

    p_write = malloc(sizeof(wiced_bt_gatt_value_t) + CCCD_LENGTH);

    if (p_write)
    {
        memset(p_write, 0, sizeof(wiced_bt_gatt_value_t) + CCCD_LENGTH);
        p_write->handle   = cts_service_handle + GATT_CCCD_HANDLE;
        p_write->len      = CCCD_LENGTH;
        p_write->value[0] = 1; //1 - enable or 0 - disable notification;

        status = wiced_bt_gatt_send_write(bt_connection_id, GATT_WRITE, p_write);
        free(p_write);
    }
    return status;
}

/*****************************************************************************************
* Function Name: print_notification_data()
******************************************************************************************
* Summary:
*   Parses the notification data to get date, time and other fields.
*
* Parameters:
*   wiced_bt_gatt_data_t notif_data: Notification packet from GATT server
*
* Return:
*   None
*
****************************************************************************************/
static void print_notification_data(wiced_bt_gatt_data_t notif_data)
{
    time_date_notif.year          = (notif_data.p_data[1] << 8u) | notif_data.p_data[0];
    time_date_notif.month         = notif_data.p_data[2];
    time_date_notif.day           = notif_data.p_data[3];
    time_date_notif.hours         = notif_data.p_data[4];
    time_date_notif.minutes       = notif_data.p_data[5];
    time_date_notif.seconds       = notif_data.p_data[6];
    time_date_notif.day_of_week   = notif_data.p_data[7];
    time_date_notif.fractions_256 = notif_data.p_data[8];
    time_date_notif.adjust_reason = notif_data.p_data[9];

    if(time_date_notif.adjust_reason)
    {
        if((time_date_notif.adjust_reason & MANUAL_TIME_UPDATE) == MANUAL_TIME_UPDATE)
        {
            printf("Time Adjust Reason: Manual Time Update\n");
        }
        if((time_date_notif.adjust_reason & EXTERNAL_REFERENCE_TIME_UPDATE) == EXTERNAL_REFERENCE_TIME_UPDATE)
        {
            printf("Time Adjust Reason: External Reference Time Update\n");
        }
        if((time_date_notif.adjust_reason & CHANGE_OF_TIME_ZONE) == CHANGE_OF_TIME_ZONE)
        {
            printf("Time Adjust Reason: Change of Time Zone\n");
        }
        if((time_date_notif.adjust_reason & CHANGE_OF_DST) == CHANGE_OF_DST)
        {
            printf("Time Adjust Reason: Change of DST\n");
        }
    }

    printf("Date (dd-mm-yyyy): %d - %d - %d \n", time_date_notif.day,
                                                 time_date_notif.month,
                                                 time_date_notif.year);
    printf("Time (HH:MM:SS): %d:%d:%d \n", time_date_notif.hours,
                                           time_date_notif.minutes,
                                           time_date_notif.seconds);
    printf("Day of the week = %s\n\n", get_day_of_week(time_date_notif.day_of_week));
}

/*************************************************************************************
* Function Name: get_day_of_week()
**************************************************************************************
* Summary: Parse day of week code to string
*
* Parameters:
*   uint8_t day: code for day of the week
*
* Return:
*   const char*: Pointer to day of week string
*
*************************************************************************************/
const char* get_day_of_week(uint8_t day)
{
    if (day >= sizeof(day_of_week_str) / sizeof(char*))
    {
        return "** UNKNOWN **";
    }

    return day_of_week_str[day];
}


/* [] END OF FILE */
