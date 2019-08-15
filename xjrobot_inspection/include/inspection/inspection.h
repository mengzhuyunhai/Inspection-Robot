/*
 * Inspection
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

/* Author: Zhang Chunyang */

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <string>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
using namespace std;
#include "ros/ros.h"
#include "ros/types.h"
#include "std_msgs/Int8.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include "common.h"

/* include msg header */
#include <xjrobot_interface_msg/taskStatus.h>   // route/taskStatus
#include <xjrobot_interface_msg/mode_code.h>    // mode
#include <xjrobot_interface_msg/status_code.h>  // status
#include <xjrobot_interface_msg/base_info.h>    // base_info
#include <xjrobot_interface_msg/Rst.h>
#include <xjrobot_interface_msg/MapStatus.h>
#include <xjrobot_interface_msg/Edge.h>
#include <xjrobot_interface_msg/RouteGraph.h>
#include <xjrobot_interior_msg/locate_state.h>    // locate_state
#include <xjrobot_interior_msg/flag.h>
#include "xjrobot_interior_msg/chargeOpen.h"
#include "geometry_msgs/Twist.h"
/* include srv header */
#include <xjrobot_interface_msg/route_routeOrder.h> // route_routeOrder
#include <xjrobot_interface_msg/route_nextOrder.h>  // route_nextOrder
#include <xjrobot_interior_msg/nav_state.h>    // nav_state
#include <xjrobot_interior_msg/nav_task.h>    // nav_task
#include <xjrobot_interior_msg/path_planning.h>    // path_planning
#include <xjrobot_interior_msg/autoCharge_chargeOrder.h>    // autoCharge_chargeOrder
#include <xjrobot_interior_msg/getPointsInfo.h>    // getPointsInfo
#include <xjrobot_interior_msg/chargeOpen.h>    // chargeOpen
#include <xjrobot_interior_msg/special_nav_status.h>

enum AUTOCHARGE_STATE
{
    AUTOCHARGE_GOING_TO_CHARGEROME,    //行驶中    
    AUTOCHARGE_REACHED_CHARGEROME,     //到达充电房门口
    AUTOCHARGE_CHARGING,               //充电中
    AUTOCHARGE_RETRY,                  //充电没有成功后的重新尝试
    AUTOCHARGE_SPARE,                  //空闲状态
    AUTOCHARGE_ERROR,                  //充电发生错误
    AUTOCHARGE_FAILED,                 //充电重新尝试一定次数后(暂时设置为10次)还是未成功,发布失败状态
    AUTOCHARGE_WAITING_DOOR_OPEN_OUTOF_TIME,   //等待充电房门打开超时(暂时设置为10分钟)
    AUTOCHARGE_CHARGE_OPEN_FAILED,             //开启充电失败
    AUTOCHARGE_NAVIGATION_OUTOF_TIME,           //导航超时(暂时设置为半个小时还未从当前点跑到下个点)
    AUTOCHARGE_PILING           //正在对桩
};

#define VISIT_NO    0
#define VISIT_YES   1

#define NAV_MOVING    0
#define NAV_STOP    1

#define NAV_STATE_MOVING    1
#define NAV_STATE_END   0
#define NAV_STATE_ERR   -1

#define TASK_STATUS_FINISH   -2
#define TASK_STATUS_ARRIVE   -1
#define TASK_STATUS_OPEN_DOOR   -3
#define TASK_STATUS_CLOSE_DOOR  -4
#define TASK_STATUS_SPECIAL_FINISH  -5

#define TASK_START  1
#define TASK_END    0

#define THREAD_DELAY_TIME   200000

#define NAV_MODE_INS    0
#define NAV_MODE_BACK   1
#define NAV_MODE_SPECIAL_BACK   2
#define NAV_MODE_SPECIAL_TASK   3

#define DOOR_STATE_CLOSE    0
#define DOOR_STATE_OPEN 1
#define DOOR_STATE_OPENNING 2
#define DOOR_STATE_CLOSING  3

#define CHARGE_UNLOCK   0
#define CHARGE_LOCK 1

#define ROBOT_IN_DOOR   0
#define ROBOT_OUT_DOOR  1
#define ROBOT_UNKNOW    2

#define CHARGE_STATUS_NEED_NOT_CHARGE   0x00
#define CHARGE_STATUS_NEED_CHARGE   0x04
#define CHARGE_STATUS_CHARGING  0x02

#define POWER_IS_RIGHT 0X00
#define POWER_IS_ERROR 0X01

#define CHARGE_HOUSE_RADIUS     1.0
#define CHARGE_HOUSE_LENGTH     1.70
#define CHANGE_HOUSE_WIDTH      1.70

typedef struct Task_Path_
{
    int path_num;
    int path_node_list[MAX_NODE_NUM];
}Task_Path;

typedef struct Task_List_
{
    int task_num;
    int task_index;
    Task_Path task_list[MAX_TASK_NUM];
}Task_List;

class Inspection
{
    public:
        Inspection(){};
        ~Inspection(){};

        int init(int argc, char **argv);
        static int takeOffPile(void);
        static int getCurrentPoint(double x, double y, int * point);
        void base_infoCallback(const xjrobot_interface_msg::base_infoConstPtr& msg);
        void modeCallback(const xjrobot_interface_msg::mode_codeConstPtr& msg);
        void nav_stateCallback(const xjrobot_interior_msg::nav_stateConstPtr& msg);
        void charrge_stateCallback(const std_msgs::Int8::ConstPtr& msg);
        bool isTaskNode(int node, int startNode, vector<int> taskList, int taskVisit[]);
		bool isTaskNode(int node, vector<int> taskList);
        static bool sendOneTask(Task_List * taskL);
        bool route_routeOrderCallback(xjrobot_interface_msg::route_routeOrder::Request  &req,
                                                                xjrobot_interface_msg::route_routeOrder::Response &res);
        bool route_nextOrderCallback(xjrobot_interface_msg::route_nextOrder::Request  &req,
                                                                xjrobot_interface_msg::route_nextOrder::Response &res);
        void locate_stateCallback(const xjrobot_interior_msg::locate_stateConstPtr& msg);
        void robot_pose_timerCallback(const ros::TimerEvent& event);
        static int initPose();
        static void robot_move_front();
        static void robot_move_stop();
        static int unlock_charge();
        int isRobotOutDoor(double x, double y, int * robot_w);
        static int one_key_to_return();
        void lineNavStateCallback(const xjrobot_interior_msg::flagConstPtr& msg);

        ros::NodeHandle n;
        pthread_t inspectionId;
        pthread_t statusId;
        pthread_t id_task_status;
        ros::Subscriber modeSub;
        ros::Subscriber nav_stateSub;
        ros::ServiceServer route_routeOrderService;
        ros::ServiceServer route_nextOrderService;
        int lastMode;
        ros::ServiceClient path_planningClient;
        ros::ServiceClient _charge_start_client;
        static int currentPoint;
        static ros::ServiceClient autoCharge_chargeOrderClient;
        static ros::Publisher doorState_;
        static std_msgs::Int8 doorOpen;
        std_msgs::Int8 doorStatePub;
        ros::Subscriber locate_stateSub;
        xjrobot_interior_msg::locate_state locate_state_;
        ros::Timer robot_pose_timer;
        xjrobot_interface_msg::base_info baseInfo;
        ros::Subscriber base_infoSub;
        ros::Subscriber charrge_stateSub;
        unsigned char lineNavState;

        static void * pTaskStatusPro(void *);
        static void * statusThread(void *);
        static void * inspectionThread(void *);
        static double W_periodValue;
        static double Y_periodValue;
        static ros::Publisher status_code_;
        static ros::Publisher taskStatus_;
        static xjrobot_interface_msg::status_code status_codePub;
        static xjrobot_interface_msg::taskStatus taskStatusPub;
        static int currentMode;
        static char inspectionStatus;
        static int TaskState;
        static int currentTaskId;
        static Task_List taskList;
        static int navMode;
        static ros::ServiceClient nav_taskClient;
        static int NavState;
        static int locate_state_value;
        static ros::ServiceClient getPointsInfoClient;
        static double robot_pose[3];
        static int base_info_state;
        static int robot_pose_state;
        static ros::Publisher _cmd_vel_pub;
        static geometry_msgs::Twist cmd_vel_;
        static ros::ServiceClient chargeOpenClient;
        static ros::Publisher _init_pose_pub;
        static geometry_msgs::PoseWithCovarianceStamped init_pose_;
        static ros::ServiceClient special_nav_statusClient;
        ros::Subscriber lineNavStateSub;
};
