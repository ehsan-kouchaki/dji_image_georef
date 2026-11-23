/**
 ********************************************************************
 * @file    test_gimbal_manager.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any log(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <widget_interaction_test/test_widget_interaction.h>
#include "test_gimbal_manager.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include "dji_gimbal_manager.h"
#include "dji_fc_subscription.h"
#include "dji_aircraft_info.h"

/* Private constants ---------------------------------------------------------*/

/*
 * Gimbal pitch sweep parameters for DjiTest_GimbalManagerSweepPitch().
 *
 *   SWEEP_START_DEG : starting pitch angle in degrees (straight down)
 *   SWEEP_END_DEG   : final pitch angle in degrees
 *   SWEEP_STEP_DEG  : how many degrees to move per step
 *   SWEEP_STEP_MS   : how many milliseconds to wait between steps
 *
 * The 'time' field passed to DjiGimbalManager_Rotate() is set equal to
 * SWEEP_STEP_MS / 1000.0, so the gimbal motor completes each small move
 * smoothly within the sleep window before the next command arrives.
 *
 * Total motion  = SWEEP_END_DEG - SWEEP_START_DEG = 60 degrees
 * Total steps   = 60 / 0.5 = 120
 * Total duration = 120 * 2000 ms = 240 s (~4 minutes)
 */
#define SWEEP_START_DEG   (-90.0f)
#define SWEEP_END_DEG     (-30.0f)
#define SWEEP_STEP_DEG    (  0.5f)
#define SWEEP_STEP_MS     ( 2000 )

/* Private types -------------------------------------------------------------*/
typedef enum {
    DJI_TEST_GIMBAL_ROTATION,
    DJI_TEST_GIMBAL_RESET,
} E_DjiTestGimbalAction;

typedef struct {
    E_DjiTestGimbalAction action;
    T_DjiGimbalManagerRotation rotation;
} T_DjiTestGimbalActionList;

/* Private values -------------------------------------------------------------*/
static const T_DjiTestGimbalActionList s_rotationActionList[] =
{
        {.action = DJI_TEST_GIMBAL_RESET},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 30,  0,   0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, -30, 0,   0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, -30, 0,   0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 30,  0,   0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   30,  0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   -30, 0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   -30, 0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   30,  0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   0,   4,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, 0,   0,   -4, 0.2},
        {.action = DJI_TEST_GIMBAL_RESET},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE, 30,  0,   0,  0.2},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE, -90, 0,   0,  0.5},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE, -60, 0,   0,  0.5},
        {.action = DJI_TEST_GIMBAL_ROTATION, .rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE, -30, 0,   0,  0.5},
        {.action = DJI_TEST_GIMBAL_RESET},
};

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

/**
 * @brief Original gimbal test sample — runs a pre-defined action list.
 *        Bound to menu option [a].
 */
T_DjiReturnCode DjiTest_GimbalManagerRunSample(E_DjiMountPosition mountPosition, E_DjiGimbalMode gimbalMode)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiGimbalManagerRotation rotation;
    T_DjiAircraftInfoBaseInfo baseInfo;
    E_DjiAircraftSeries aircraftSeries;

    returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to get aircraft base info, return code 0x%08X", returnCode);
        goto out;
    }

    aircraftSeries = baseInfo.aircraftSeries;

    USER_LOG_INFO("Gimbal manager sample start");
    DjiTest_WidgetLogAppend("Gimbal manager sample start");

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to subscribe topic %d, 0x%08X",
                       DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 1: Init gimbal manager module");
    DjiTest_WidgetLogAppend("--> Step 1: Init gimbal manager module");
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init gimbal manager failed, error code: 0x%08X", returnCode);
        goto out;
    }

    if (gimbalMode == DJI_GIMBAL_MODE_FREE) {
        USER_LOG_INFO("--> Step 2: Set gimbal to free mode");
        DjiTest_WidgetLogAppend("--> Step 2: Set gimbal to free mode");
    } else if (gimbalMode == DJI_GIMBAL_MODE_YAW_FOLLOW) {
        USER_LOG_INFO("--> Step 2: Set gimbal to yaw follow mode");
        DjiTest_WidgetLogAppend("--> Step 2: Set gimbal to yaw follow mode");
    }
    returnCode = DjiGimbalManager_SetMode(mountPosition, gimbalMode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set gimbal mode failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 3: Reset gimbal angles.\r\n");
    returnCode = DjiGimbalManager_Reset(mountPosition, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
    }

    USER_LOG_INFO("--> Step 4: Rotate gimbal to target angle by action list\r\n");
    for (int i = 0; i < sizeof(s_rotationActionList) / sizeof(T_DjiTestGimbalActionList); ++i) {
        if (s_rotationActionList[i].action == DJI_TEST_GIMBAL_RESET) {
            USER_LOG_INFO("Target gimbal reset.\r\n");
            returnCode = DjiGimbalManager_Reset(mountPosition, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
            }
            osalHandler->TaskSleepMs(2000);
        } else if (s_rotationActionList[i].action == DJI_TEST_GIMBAL_ROTATION) {

            if (gimbalMode == DJI_GIMBAL_MODE_FREE &&
                s_rotationActionList[i].rotation.rotationMode == DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE) {
                continue;
            }

            rotation = s_rotationActionList[i].rotation;

            if (DJI_AIRCRAFT_SERIES_M30 == aircraftSeries
                || DJI_AIRCRAFT_SERIES_M3 == aircraftSeries
                || DJI_AIRCRAFT_SERIES_M3D == aircraftSeries
                || DJI_AIRCRAFT_SERIES_M4 == aircraftSeries
                || DJI_AIRCRAFT_SERIES_M4D == aircraftSeries
            ) {
                if (s_rotationActionList[i].rotation.rotationMode == DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE) {
                    T_DjiFcSubscriptionGimbalAngles gimbalAngles = {0};
                    T_DjiDataTimestamp timestamp = {0};
                    returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                     (uint8_t *) &gimbalAngles,
                                     sizeof(T_DjiFcSubscriptionGimbalAngles),
                                     &timestamp);
                    rotation.yaw = gimbalAngles.z;
                }
            }

            USER_LOG_INFO("Target gimbal pry = (%.1f, %.1f, %.1f)",
                          rotation.pitch, rotation.roll, rotation.yaw);

            returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: 0x%08X",
                               s_rotationActionList[i].rotation.pitch,
                               s_rotationActionList[i].rotation.roll,
                               s_rotationActionList[i].rotation.yaw,
                               returnCode);
            }
            osalHandler->TaskSleepMs(1000);
        }
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to unsubscribe topic %d, 0x%08X",
                       DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, returnCode);
    }

    USER_LOG_INFO("--> Step 5: Deinit gimbal manager module");
    DjiTest_WidgetLogAppend("--> Step 5: Deinit gimbal manager module");
    returnCode = DjiGimbalManager_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit gimbal manager failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Gimbal manager sample end");
    return returnCode;
}

/**
 * @brief Sweep the gimbal pitch from SWEEP_START_DEG (-90°) to SWEEP_END_DEG (-30°),
 *        moving SWEEP_STEP_DEG (+0.5°) every SWEEP_STEP_MS (2000 ms).
 *
 *        Total motion  : 60 degrees
 *        Total steps   : 120
 *        Total duration: ~4 minutes
 *
 *        The function uses ABSOLUTE angle mode so the target angle is always
 *        exact regardless of where the gimbal currently sits. The 'time' field
 *        is set to SWEEP_STEP_MS/1000 seconds so the motor finishes the small
 *        move smoothly within the sleep window.
 *
 *        RC image capture is fully independent of PSDK gimbal control —
 *        the pilot may press the shutter freely throughout the sweep.
 *
 * @param mountPosition  Payload port the gimbal is attached to (use 1 for
 *                       the main downward gimbal on the M300 RTK).
 * @return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS if the sweep completed
 *         without errors, or the first non-success error code encountered.
 */
T_DjiReturnCode DjiTest_GimbalManagerSweepPitch(E_DjiMountPosition mountPosition)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiGimbalManagerRotation rotation = {0};

    USER_LOG_INFO("Gimbal pitch sweep start: %.1f deg -> %.1f deg, "
                  "%.1f deg/step every %d ms (~%.0f s total)",
                  SWEEP_START_DEG, SWEEP_END_DEG, SWEEP_STEP_DEG, SWEEP_STEP_MS,
                  ((SWEEP_END_DEG - SWEEP_START_DEG) / SWEEP_STEP_DEG) * (SWEEP_STEP_MS / 1000.0f));

    /* --- Step 1: Init gimbal manager --------------------------------------- */
    USER_LOG_INFO("--> Step 1: Init gimbal manager module");
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init gimbal manager failed: 0x%08X", returnCode);
        return returnCode;
    }

    /* --- Step 2: Set YAW_FOLLOW mode (standard operating mode for M300) --- */
    USER_LOG_INFO("--> Step 2: Set gimbal to yaw-follow mode");
    returnCode = DjiGimbalManager_SetMode(mountPosition, DJI_GIMBAL_MODE_YAW_FOLLOW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set gimbal mode failed: 0x%08X", returnCode);
        goto cleanup;
    }

    
    /* Read current gimbal yaw so we never change it during the sweep */
    T_DjiFcSubscriptionGimbalAngles gimbalAngles = {0};
    T_DjiDataTimestamp timestamp = {0};

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to subscribe gimbal angles: 0x%08X", returnCode);
        goto cleanup;
    }
    osalHandler->TaskSleepMs(200); /* brief wait for first data to arrive */

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                         (uint8_t *) &gimbalAngles,
                                                         sizeof(T_DjiFcSubscriptionGimbalAngles),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to get gimbal angles: 0x%08X", returnCode);
        goto cleanup;
    }

    dji_f32_t currentYaw = gimbalAngles.z;
    USER_LOG_INFO("Current gimbal yaw = %.1f deg — will be held throughout sweep", currentYaw);
    
    
    /* --- Step 3: Move to start position (-90 degrees) --------------------- */
    USER_LOG_INFO("--> Step 3: Move to start position (pitch = %.1f deg)", SWEEP_START_DEG);
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rotation.pitch        = SWEEP_START_DEG;
    rotation.roll         = 0.0f;
    rotation.yaw          = currentYaw;
    rotation.time         = 3.0;   /* allow 3 s for the initial move to -90 */

    returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Move to start position failed: 0x%08X", returnCode);
        goto cleanup;
    }

    /* Wait for the initial move to complete before starting the sweep */
    osalHandler->TaskSleepMs(3500);

    /* --- Step 4: Sweep from SWEEP_START_DEG to SWEEP_END_DEG -------------- */
    USER_LOG_INFO("--> Step 4: Starting pitch sweep...");

    dji_f32_t currentPitch = SWEEP_START_DEG;
    int stepNumber = 0;

    while (currentPitch <= SWEEP_END_DEG) {

        rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
        rotation.pitch        = currentPitch;
        rotation.roll         = 0.0f;
        rotation.yaw          = currentYaw;
        rotation.time         = (dji_f64_t)(SWEEP_STEP_MS) / 1000.0; /* 2.0 s */

        USER_LOG_INFO("  Step %3d: pitch = %.1f deg", stepNumber, currentPitch);

        returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("  Rotate to %.1f deg failed: 0x%08X", currentPitch, returnCode);
            /* Non-fatal: log and continue to next step */
        }

        /* Wait the full step interval so the motor finishes before next command */
        osalHandler->TaskSleepMs(SWEEP_STEP_MS);

        currentPitch += SWEEP_STEP_DEG;
        stepNumber++;
    }

    USER_LOG_INFO("Gimbal pitch sweep complete. Final angle: %.1f deg", SWEEP_END_DEG);

cleanup:
    /* --- Step 5: Deinit gimbal manager ------------------------------------ */
    USER_LOG_INFO("--> Step 5: Deinit gimbal manager module");
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    T_DjiReturnCode deinitCode = DjiGimbalManager_Deinit();
    if (deinitCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit gimbal manager failed: 0x%08X", deinitCode);
    }

    return returnCode;
}

/* Private functions definition-----------------------------------------------*/
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
