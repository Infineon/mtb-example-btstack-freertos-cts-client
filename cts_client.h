/******************************************************************************
* File Name: ctc_client.c
*
* Description: This file contains macros, structures, enumerations and function
*              prototypes used in cts_client.c file.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_bt_dev.h"
#include <FreeRTOS.h>
#include <task.h>

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* Macros for button interrupt and button task */
/* Interrupt priority for the GPIO connected to the user button */
#define BUTTON_INTERRUPT_PRIORITY       (7u)
#define BUTTON_TASK_PRIORITY            (configMAX_PRIORITIES - 1)
#define BUTTON_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 8)

/*******************************************************************************
*        Enumerations
*******************************************************************************/
/* This enumeration represents the postions of bitfield in 'adjust reason' field
   of notification */
typedef enum
{
    MANUAL_TIME_UPDATE = 0x01,
    EXTERNAL_REFERENCE_TIME_UPDATE = 0x02,
    CHANGE_OF_TIME_ZONE = 0x04,
    CHANGE_OF_DST = 0x08,
}adjust_reason_bits_t;

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
} current_time_data_t;

typedef struct
{
    uint16_t cts_start_handle;
    uint16_t cts_end_handle;
    uint16_t cts_char_handle;
    uint16_t cts_char_val_handle;
    uint16_t cts_cccd_handle;
    bool cts_service_found;
} cts_discovery_data_t;
/*******************************************************************************
 * Extern variables
 ******************************************************************************/
extern TaskHandle_t  button_task_handle;

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
/* FreeRTOS task functions */
void button_task (void *pvParameters);

/* Callback function for Bluetooth stack management events */
wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                                 wiced_bt_management_evt_data_t *p_event_data);
