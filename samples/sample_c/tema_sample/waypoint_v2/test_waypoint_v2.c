/**
 ********************************************************************
 * @file    test_waypoint_v2.c
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
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <widget_interaction_test/test_widget_interaction.h>
#include "test_waypoint_v2.h"
#include "dji_waypoint_v2.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/

/*
 * Path to the waypoints CSV file on the Khadas onboard computer.
 * Copy your mission file here before running the program:
 *   scp waypoints.csv khadas@<ip>:/home/khadas/waypoints.csv
 *
 * File format — one waypoint per line, no header row:
 *   latitude_deg, longitude_deg, altitude_m, speed_m_per_s
 *
 * Example:
 *   37.774900, -122.419400, 30.0, 3.0
 *   37.774950, -122.419100, 30.0, 3.0
 *
 * Rules:
 *   - latitude and longitude are decimal degrees (WGS84)
 *   - altitude is meters ABOVE the takeoff point (relative height, not sea level)
 *   - speed is the cruise speed in m/s for that leg (max 9 m/s)
 *   - Lines starting with # are treated as comments and ignored
 *   - At least 2 waypoints are required
 */
#define WAYPOINTS_FILE_PATH  "/home/khadas/waypoints.csv"

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t eventID;
    char *eventStr;
} T_DjiTestWaypointV2EventStr;

typedef struct {
    uint8_t missionState;
    char *stateStr;
} T_DjiTestWaypointV2StateStr;

/* Private values -------------------------------------------------------------*/
static T_DjiOsalHandler *osalHandler = NULL;
static uint32_t s_missionID = 12345;

/*
 * The SDK holds an internal pointer to the waypoint array after
 * DjiWaypointV2_UploadMission() is called — it does NOT copy the data.
 * So we must keep the array alive for the entire lifetime of the mission
 * and only free it after DjiWaypointV2_Deinit() has been called.
 * Using a static pointer here ensures that lifetime is correctly managed.
 */
static T_DjiWaypointV2 *s_waypointList = NULL;

/* reference note of "T_DjiWaypointV2MissionEventPush" */
static const T_DjiTestWaypointV2EventStr s_waypointV2EventStr[] = {
    {.eventID = 0x01, .eventStr = "Interrupt Event"},
    {.eventID = 0x02, .eventStr = "Resume Event"},
    {.eventID = 0x03, .eventStr = "Stop Event"},
    {.eventID = 0x10, .eventStr = "Arrival Event"},
    {.eventID = 0x11, .eventStr = "Finished Event"},
    {.eventID = 0x12, .eventStr = "Avoid Obstacle Event"},
    {.eventID = 0x30, .eventStr = "Action Switch Event"},
    {.eventID = 0xFF, .eventStr = "Unknown"}
};

/* reference note of "T_DjiWaypointV2MissionStatePush" */
static const T_DjiTestWaypointV2StateStr s_waypointV2StateStr[] = {
    {.missionState = 0x00, .stateStr = "Ground station not start"},
    {.missionState = 0x01, .stateStr = "Mission prepared"},
    {.missionState = 0x02, .stateStr = "Enter mission"},
    {.missionState = 0x03, .stateStr = "Execute mission"},
    {.missionState = 0x04, .stateStr = "Pause Mission"},
    {.missionState = 0x05, .stateStr = "Enter mission after ending pause"},
    {.missionState = 0x06, .stateStr = "Exit mission"},
    {.missionState = 0xFF, .stateStr = "Unknown"}
};

/* Private functions declaration ---------------------------------------------*/
static uint8_t DjiTest_WaypointV2GetMissionEventIndex(uint8_t eventID);
static uint8_t DjiTest_WaypointV2GetMissionStateIndex(uint8_t state);
static T_DjiWaypointV2 *DjiTest_WaypointV2LoadFromFile(const char *filePath, uint16_t *count);
static T_DjiReturnCode DjiTest_WaypointV2UploadMission(void);
static T_DjiReturnCode DjiTest_WaypointV2EventCallback(T_DjiWaypointV2MissionEventPush eventData);
static T_DjiReturnCode DjiTest_WaypointV2StateCallback(T_DjiWaypointV2MissionStatePush stateData);
static T_DjiReturnCode DjiTest_WaypointV2Init(void);
static T_DjiReturnCode DjiTest_WaypointV2DeInit(void);

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_WaypointV2RunSample(void)
{
    T_DjiReturnCode returnCode;
    uint32_t timeOutMs = 1000;
    T_DjiWaypointV2GlobalCruiseSpeed setGlobalCruiseSpeed = 0;
    T_DjiWaypointV2GlobalCruiseSpeed getGlobalCruiseSpeed = 0;

    USER_LOG_INFO("Waypoint V2 sample start");
    DjiTest_WidgetLogAppend("Waypoint V2 sample start");

    USER_LOG_INFO("--> Step 1: Init Waypoint V2 sample");
    DjiTest_WidgetLogAppend("--> Step 1: Init Waypoint V2 sample");
    returnCode = DjiTest_WaypointV2Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init waypoint V2 sample failed, error code: 0x%08X", returnCode);
        USER_LOG_INFO("Waypoint V2 sample end");
        return returnCode;
    }

    USER_LOG_INFO("--> Step 2: Subscribe GPS fused data");
    DjiTest_WidgetLogAppend("--> Step 2: Subscribe GPS fused data");
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe GPS fused data failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 3: Register waypoint V2 event and state callback\r\n");
    DjiTest_WidgetLogAppend("--> Step 3: Register waypoint V2 event and state callback\r\n");
    returnCode = DjiWaypointV2_RegisterMissionEventCallback(DjiTest_WaypointV2EventCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register waypoint V2 event failed, error code: 0x%08X", returnCode);
        goto out;
    }
    returnCode = DjiWaypointV2_RegisterMissionStateCallback(DjiTest_WaypointV2StateCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register waypoint V2 state failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(timeOutMs);

    USER_LOG_INFO("--> Step 4: Upload waypoint V2 mission from file: %s\r\n", WAYPOINTS_FILE_PATH);
    DjiTest_WidgetLogAppend("--> Step 4: Upload waypoint V2 mission\r\n");
    returnCode = DjiTest_WaypointV2UploadMission();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Upload waypoint V2 mission failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(timeOutMs);

    USER_LOG_INFO("--> Step 5: Start waypoint V2 mission\r\n");
    DjiTest_WidgetLogAppend("--> Step 5: Start waypoint V2 mission\r\n");
    returnCode = DjiWaypointV2_Start();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start waypoint V2 mission failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(20 * timeOutMs);

    USER_LOG_INFO("--> Step 6: Set global cruise speed\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Set global cruise speed\r\n");
    setGlobalCruiseSpeed = 1.5;
    returnCode = DjiWaypointV2_SetGlobalCruiseSpeed(setGlobalCruiseSpeed);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set global cruise speed failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(timeOutMs);

    USER_LOG_INFO("--> Step 7: Get global cruise speed\r\n");
    DjiTest_WidgetLogAppend("--> Step 7: Get global cruise speed\r\n");
    returnCode = DjiWaypointV2_GetGlobalCruiseSpeed(&getGlobalCruiseSpeed);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get global cruise speed failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current global cruise speed is %f m/s", getGlobalCruiseSpeed);
    osalHandler->TaskSleepMs(timeOutMs);

    USER_LOG_INFO("--> Step 8: Pause waypoint V2 for 5 s\r\n");
    DjiTest_WidgetLogAppend("--> Step 8: Pause waypoint V2 for 5 s\r\n");
    returnCode = DjiWaypointV2_Pause();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Pause waypoint V2 failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(5 * timeOutMs);

    USER_LOG_INFO("--> Step 9: Resume waypoint V2\r\n");
    DjiTest_WidgetLogAppend("--> Step 9: Resume waypoint V2\r\n");
    returnCode = DjiWaypointV2_Resume();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Resume waypoint V2 failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(50 * timeOutMs);

    USER_LOG_INFO("--> Step 10: Deinit Waypoint V2 sample\r\n");
    DjiTest_WidgetLogAppend("--> Step 10: Deinit Waypoint V2 sample\r\n");
out:
    returnCode = DjiTest_WaypointV2DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit waypoint V2 sample failed, error code: 0x%08X", returnCode);
    }

    USER_LOG_INFO("Waypoint V2 sample end");
    DjiTest_WidgetLogAppend("Waypoint V2 sample end");

    return returnCode;
}

/* Private functions definition ----------------------------------------------*/

/**
 * @brief Read waypoints from a plain-text CSV file.
 *
 * File format (one waypoint per line):
 *   latitude_deg, longitude_deg, altitude_m, speed_m_per_s
 *
 * Lines starting with '#' or blank lines are ignored.
 *
 * IMPORTANT: The PSDK Waypoint V2 API requires latitude and longitude
 * in RADIANS, not degrees. This function converts automatically.
 *
 * @param filePath  Absolute path to the CSV file on the filesystem.
 * @param count     Output: number of waypoints successfully loaded.
 * @return          Malloc'd array of T_DjiWaypointV2 structs, or NULL on error.
 *                  Caller must Free() this pointer when done.
 */
static T_DjiWaypointV2 *DjiTest_WaypointV2LoadFromFile(const char *filePath, uint16_t *count)
{
    *count = 0;

    FILE *f = fopen(filePath, "r");
    if (f == NULL) {
        USER_LOG_ERROR("Cannot open waypoints file: %s  -- "
                       "Copy your CSV to the Khadas first: "
                       "scp waypoints.csv khadas@<ip>:%s", filePath, filePath);
        return NULL;
    }

    /* First pass: count valid (non-comment, non-blank) lines */
    uint16_t lineCount = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
        lineCount++;
    }

    if (lineCount == 0) {
        USER_LOG_ERROR("Waypoints file is empty or has only comments: %s", filePath);
        fclose(f);
        return NULL;
    }

    T_DjiWaypointV2 *list = (T_DjiWaypointV2 *) osalHandler->Malloc(lineCount * sizeof(T_DjiWaypointV2));
    if (list == NULL) {
        USER_LOG_ERROR("Malloc failed allocating waypoint list (%d waypoints)", lineCount);
        fclose(f);
        return NULL;
    }
    memset(list, 0, lineCount * sizeof(T_DjiWaypointV2));

    /* Second pass: parse each valid line */
    rewind(f);
    uint16_t idx = 0;
    uint16_t lineNum = 0;
    while (fgets(line, sizeof(line), f)) {
        lineNum++;
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;

        double lat_deg, lon_deg, alt_m, speed_ms;
        int parsed = sscanf(line, " %lf , %lf , %lf , %lf", &lat_deg, &lon_deg, &alt_m, &speed_ms);
        if (parsed != 4) {
            USER_LOG_ERROR("Skipping malformed line %d in %s: [%s]", lineNum, filePath, line);
            continue;
        }

        /* Validate reasonable ranges */
        if (lat_deg < -90.0 || lat_deg > 90.0) {
            USER_LOG_ERROR("Line %d: latitude %f is out of range [-90, 90], skipping", lineNum, lat_deg);
            continue;
        }
        if (lon_deg < -180.0 || lon_deg > 180.0) {
            USER_LOG_ERROR("Line %d: longitude %f is out of range [-180, 180], skipping", lineNum, lon_deg);
            continue;
        }
        if (alt_m < 0.0 || alt_m > 3000.0) {
            USER_LOG_ERROR("Line %d: altitude %f is out of safe range [0, 3000] m, skipping", lineNum, alt_m);
            continue;
        }
        if (speed_ms < 0.5 || speed_ms > 9.0) {
            USER_LOG_ERROR("Line %d: speed %f is out of range [0.5, 9.0] m/s, skipping", lineNum, speed_ms);
            continue;
        }

        /* PSDK V2 API requires radians for lat/lon */
        list[idx].latitude       = lat_deg * M_PI / 180.0;
        list[idx].longitude      = lon_deg * M_PI / 180.0;
        list[idx].relativeHeight = (dji_f32_t) alt_m;
        list[idx].autoFlightSpeed = (dji_f32_t) speed_ms;
        list[idx].maxFlightSpeed  = 9.0f;

        /* Flight behaviour settings */
        list[idx].waypointType  = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
        list[idx].headingMode   = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
        list[idx].config.useLocalMaxVel    = 0;
        list[idx].config.useLocalCruiseVel = 0;
        list[idx].dampingDistance = 40;
        list[idx].heading   = 0;
        list[idx].turnMode  = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;
        list[idx].pointOfInterest.positionX = 0;
        list[idx].pointOfInterest.positionY = 0;
        list[idx].pointOfInterest.positionZ = 0;

        USER_LOG_INFO("  Waypoint %d: lat=%.6f lon=%.6f alt=%.1fm speed=%.1fm/s",
                      idx, lat_deg, lon_deg, alt_m, speed_ms);
        idx++;
    }

    fclose(f);

    if (idx < 2) {
        USER_LOG_ERROR("Need at least 2 valid waypoints, only got %d from %s", idx, filePath);
        osalHandler->Free(list);
        return NULL;
    }

    USER_LOG_INFO("Loaded %d waypoints from %s", idx, filePath);
    *count = idx;
    return list;
}

/**
 * @brief Load the waypoints CSV file and upload the mission to the drone.
 *        No actions are attached — the drone simply flies the route.
 *
 * NOTE on memory: DjiWaypointV2_UploadMission() does NOT copy the waypoint
 * array internally — it retains a pointer to it. Therefore we store the
 * array in the module-level static pointer s_waypointList and free it only
 * inside DjiTest_WaypointV2DeInit(), AFTER DjiWaypointV2_Deinit() has
 * released the SDK's internal reference. Freeing it here (immediately after
 * upload) causes the double-free crash that was observed previously.
 */
static T_DjiReturnCode DjiTest_WaypointV2UploadMission(void)
{
    T_DjiReturnCode returnCode;

    /* Free any leftover list from a previous call before allocating a new one */
    if (s_waypointList != NULL) {
        osalHandler->Free(s_waypointList);
        s_waypointList = NULL;
    }

    uint16_t waypointCount = 0;
    s_waypointList = DjiTest_WaypointV2LoadFromFile(WAYPOINTS_FILE_PATH, &waypointCount);
    if (s_waypointList == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    T_DjiWayPointV2MissionSettings mission = {0};
    mission.missionID    = s_missionID + 10;
    USER_LOG_DEBUG("Mission ID: %d", mission.missionID);
    mission.repeatTimes  = 1;
    mission.finishedAction = DJI_WAYPOINT_V2_FINISHED_GO_HOME;
    mission.maxFlightSpeed  = 10;
    mission.autoFlightSpeed = 3;
    mission.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
    mission.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;
    mission.mission      = s_waypointList;   /* SDK holds this pointer — do NOT free until DeInit */
    mission.missTotalLen = waypointCount;

    /* No automated actions — drone just flies the route */
    mission.actionList.actions   = NULL;
    mission.actionList.actionNum = 0;

    returnCode = DjiWaypointV2_UploadMission(&mission);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Upload waypoint V2 mission failed, error code: 0x%08X", returnCode);
        s_waypointList = NULL;   // SDK already freed it — do NOT call Free()
    } else {
        s_waypointList = NULL;   // SDK now owns it — relinquish our reference
    }
    return returnCode;
}

static uint8_t DjiTest_WaypointV2GetMissionEventIndex(uint8_t eventID)
{
    uint8_t i;
    for (i = 0; i < sizeof(s_waypointV2EventStr) / sizeof(T_DjiTestWaypointV2EventStr); i++) {
        if (s_waypointV2EventStr[i].eventID == eventID) return i;
    }
    return i;
}

static uint8_t DjiTest_WaypointV2GetMissionStateIndex(uint8_t state)
{
    uint8_t i;
    for (i = 0; i < sizeof(s_waypointV2StateStr) / sizeof(T_DjiTestWaypointV2StateStr); i++) {
        if (s_waypointV2StateStr[i].missionState == state) return i;
    }
    return i;
}

static T_DjiReturnCode DjiTest_WaypointV2EventCallback(T_DjiWaypointV2MissionEventPush eventData)
{
    if (eventData.event == 0x01) {
        USER_LOG_INFO("[%s]: Mission interrupted, reason: 0x%x",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.interruptReason);
    } else if (eventData.event == 0x02) {
        USER_LOG_INFO("[%s]: Mission recover reason: 0x%x",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.recoverProcess);
    } else if (eventData.event == 0x03) {
        USER_LOG_INFO("[%s]: Mission exit reason: 0x%x",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.exitReason);
    } else if (eventData.event == 0x10) {
        USER_LOG_INFO("[%s]: Arrived at waypoint index %d",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.waypointIndex);
    } else if (eventData.event == 0x11) {
        USER_LOG_INFO("[%s]: Mission execute count: %d",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes);
    } else if (eventData.event == 0x12) {
        USER_LOG_INFO("[%s]: Avoid obstacle state: %d",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.avoidState);
    } else if (eventData.event == 0x30) {
        USER_LOG_INFO("[%s]: action id:%d, pre state:%d, cur state:%d, result:0x%08llX",
                      s_waypointV2EventStr[DjiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.T_DjiWaypointV2ActionExecEvent.actionId,
                      eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState,
                      eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState,
                      eventData.data.T_DjiWaypointV2ActionExecEvent.result);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_WaypointV2StateCallback(T_DjiWaypointV2MissionStatePush stateData)
{
    static uint32_t curMs = 0;
    static uint32_t preMs = 0;
    osalHandler->GetTimeMs(&curMs);
    if (curMs - preMs >= 1000) {
        preMs = curMs;
        USER_LOG_INFO("[Waypoint Index:%d]: State: %s, velocity:%.2f m/s",
                      stateData.curWaypointIndex,
                      s_waypointV2StateStr[DjiTest_WaypointV2GetMissionStateIndex(stateData.state)].stateStr,
                      (dji_f32_t) stateData.velocity / 100);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_WaypointV2Init(void)
{
    T_DjiReturnCode returnCode;

    osalHandler = DjiPlatform_GetOsalHandler();
    if (!osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init waypoint V2 data subscription module error, stat:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiWaypointV2_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init waypoint V2 module error, stat:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_WaypointV2DeInit(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFcSubscription_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit waypoint V2 data subscription module error, stat:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiWaypointV2_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit waypoint V2 module error, stat:0x%08llX", returnCode);
        return returnCode;
    }

    /*
     * Only free the waypoint list here — AFTER DjiWaypointV2_Deinit() has
     * released the SDK's internal reference to it. Freeing it any earlier
     * causes a double-free crash.
     */
    //if (s_waypointList != NULL) {
    //    osalHandler->Free(s_waypointList);
    //    s_waypointList = NULL;
   // }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
