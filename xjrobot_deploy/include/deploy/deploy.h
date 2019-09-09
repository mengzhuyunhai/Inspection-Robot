/*
 * Deploy
 * Copyright (c) 2008, www.cdjdgm.com, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Li Zhongfa */

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <string>
#include <pthread.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <vector>
#include "ros/ros.h"
#include "ros/types.h"
#include "common.h"
using namespace std;

/* include msg header */
#include <xjrobot_interface_msg/Rst.h>
#include <xjrobot_interface_msg/status_code.h>  // status
#include <xjrobot_interior_msg/nav_state.h>    // nav_state
/* include srv header */
#include <xjrobot_interface_msg/deploy_input_file.h>    // deploy_input_file
#include <xjrobot_interface_msg/deploy_travel.h>    // deploy_travel
#include <xjrobot_interface_msg/mode_code.h>    // mode
#include <xjrobot_interior_msg/xjrobot_point.h>    // xjrobot_point
#include <xjrobot_interior_msg/nav_task.h>    // nav_task
#include <xjrobot_interior_msg/path_planning.h>    // path_planning

#define DEPLOY_INPUT_PATH   "/robot/deploy/"

#define NAV_STATE_MOVING    1
#define NAV_STATE_END   0
#define NAV_STATE_ERR   -1

#define THREAD_DELAY_TIME   200000

#define UNITED  0
#define UNUNITED    1

class Deploy
{
    public:
        Deploy(){};
        ~Deploy(){};

        int init(int argc, char **argv);
        int doSystem(const char * cmd);
        void modeCallback(const xjrobot_interface_msg::mode_codeConstPtr& msg);
        void nav_stateCallback(const xjrobot_interior_msg::nav_stateConstPtr& msg);
        bool deploy_inputFileCallback(xjrobot_interface_msg::deploy_input_file::Request  &req,
                                                                xjrobot_interface_msg::deploy_input_file::Response &res);
        bool deploy_travelCallback(xjrobot_interface_msg::deploy_travel::Request  &req,
                                                        xjrobot_interface_msg::deploy_travel::Response &res);

        ros::NodeHandle n;
        pthread_t deployId;
        pthread_t statusId;
        ros::Subscriber modeSub;
        ros::Subscriber nav_stateSub;
        ros::ServiceServer deploy_input_fileService;
        ros::ServiceServer deploy_travelService;
        int lastMode;
        int NavState;
        vector<int> RoutePath;
        ros::ServiceClient nav_taskClient;
        ros::ServiceClient path_planningClient;

        static void * deployThread(void *);
        static void * statusThread(void *);
        static int currentMode;
        static char deployStatus;
        static double W_periodValue;
        static ros::Publisher status_code_;
        static xjrobot_interface_msg::status_code status_codePub;
};
