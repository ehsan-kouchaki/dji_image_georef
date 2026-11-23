/**
 ********************************************************************
 * @file    main.cpp
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
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"
#include "waypoint_v2/test_waypoint_v2.h"
#include <thread>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

start:
    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [a] Gimbal manager sample - you can control gimbal by PSDK                                       |\n"
        << "| [b] Camera stream view sample - display the camera video stream                                  |\n"
        << "| [c] Stereo vision view sample - display the stereo image                                         |\n"
        << "| [d] Run camera manager sample - you can test camera's functions interactively                    |\n"
	<< "| [e] Run waypoint V2 mission - read waypoints.csv and fly autonomously                            |\n"
	<< "| [f] Gimbal pitch sweep: -90 deg to -30 deg, +0.5 deg every 2 s          |\n"

        << std::endl;

    std::cin >> inputChar;
    switch (inputChar) {
        case 'a':
            DjiUser_RunGimbalManagerSample();
            break;
        case 'b':
            DjiUser_RunCameraStreamViewSample();
            break;
        case 'c':
            DjiUser_RunStereoVisionViewSample();
            break;
        case 'd':
            DjiUser_RunCameraManagerSample();
            break;
        default:
            break;
        //case 'e': {
	    // Launch image downloader in a background thread
	//    std::thread downloadThread([]() {
	//	DjiUser_RunCameraManagerSample(); // runs its own while(1) loop
	//    });
	//    downloadThread.detach(); // let it run independently

	    // Now run the waypoint mission in the foreground
	//    DjiTest_WaypointV2RunSample();
	//    break;
	//    }
        case 'e':
	    DjiTest_WaypointV2RunSample();
	    break;
	case 'f':
    	    DjiTest_GimbalManagerSweepPitch(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
    	    break;
    }

    osalHandler->TaskSleepMs(2000);

    goto start;
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
