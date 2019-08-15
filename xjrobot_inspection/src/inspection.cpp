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

#include "inspection.h"

double Inspection::W_periodValue;
double Inspection::Y_periodValue;
ros::Publisher Inspection::status_code_;
ros::Publisher Inspection::taskStatus_;
xjrobot_interface_msg::status_code Inspection::status_codePub;
xjrobot_interface_msg::taskStatus Inspection::taskStatusPub;
int Inspection::currentMode;
char Inspection::inspectionStatus;
int Inspection::TaskState;
int Inspection::currentTaskId;
Task_List Inspection::taskList;
int Inspection::navMode;
int Inspection::NavState;
ros::ServiceClient Inspection::nav_taskClient;
int Inspection::locate_state_value;
ros::ServiceClient Inspection::getPointsInfoClient;
double Inspection::robot_pose[3];
int Inspection::base_info_state;
int Inspection::robot_pose_state;
ros::Publisher Inspection::_cmd_vel_pub;
geometry_msgs::Twist Inspection::cmd_vel_;
ros::ServiceClient Inspection::chargeOpenClient;
ros::Publisher Inspection::_init_pose_pub;
geometry_msgs::PoseWithCovarianceStamped Inspection::init_pose_;
ros::ServiceClient Inspection::special_nav_statusClient;
int Inspection::currentPoint;
ros::ServiceClient Inspection::autoCharge_chargeOrderClient;
ros::Publisher Inspection::doorState_;
std_msgs::Int8 Inspection::doorOpen;

int Inspection::init(int argc, char **argv)
{
    int rst = RST_OK;

    lastMode = MODE_IDLE;
    currentMode = MODE_IDLE;
    NavState = NAV_STATE_END;
    TaskState = TASK_END;
    currentTaskId = 0;
    inspectionStatus = INSPECTION_STATUS_IDEL;
    navMode = NAV_MODE_INS;
    locate_state_value = LOCATE_STATE_NO_DATA;
    base_info_state = NO_DATA;
    robot_pose_state = STATE_FALSE;
    lineNavState = BEGAIN;

    base_infoSub = n.subscribe("/base_info", 10, &Inspection::base_infoCallback, this);
    autoCharge_chargeOrderClient = n.serviceClient<xjrobot_interior_msg::autoCharge_chargeOrder>("autoCharge_chargeOrder");
    path_planningClient = n.serviceClient<xjrobot_interior_msg::path_planning>("path_planning");
    nav_taskClient = n.serviceClient<xjrobot_interior_msg::nav_task>("nav_task");
    status_code_ = n.advertise<xjrobot_interface_msg::status_code>("status", 10);
    taskStatus_ = n.advertise<xjrobot_interface_msg::taskStatus>("taskStatus", 10);
    doorState_= n.advertise<std_msgs::Int8>("door_state", 10);
    modeSub = n.subscribe("/mode", 10, &Inspection::modeCallback, this);
    nav_stateSub = n.subscribe("/nav_state", 10, &Inspection::nav_stateCallback, this);
    _charge_start_client = n.serviceClient<xjrobot_interior_msg::chargeOpen>("chargeOpen");
    route_routeOrderService = n.advertiseService("route_routeOrder", &Inspection::route_routeOrderCallback, this);
    route_nextOrderService = n.advertiseService("route_nextOrder", &Inspection::route_nextOrderCallback, this);
    locate_stateSub = n.subscribe("/locate_state", 10, &Inspection::locate_stateCallback, this);
    getPointsInfoClient = n.serviceClient<xjrobot_interior_msg::getPointsInfo>("getPointsInfo");
    chargeOpenClient = n.serviceClient<xjrobot_interior_msg::chargeOpen>("chargeOpen");
    robot_pose_timer = n.createTimer(ros::Duration(0.3), &Inspection::robot_pose_timerCallback, this);
    _cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5000);
    charrge_stateSub = n.subscribe("xjrobot_autocharge/charge_state", 10, &Inspection::charrge_stateCallback, this);
    _init_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    special_nav_statusClient = n.serviceClient<xjrobot_interior_msg::special_nav_status>("xjrobot_inspection/Find_min_points");
    lineNavStateSub = n.subscribe("/navigation/goal_state", 10, &Inspection::lineNavStateCallback, this);

    rst = pthread_create(&inspectionId, NULL, inspectionThread, NULL);
    if(rst != 0)
    {
        ROS_ERROR("[xjrobot_inspection] 1.ERROR : create inspectionThread thread fail, exit");
        return rst;
    }

    rst = pthread_create(&statusId, NULL, statusThread, NULL);
    if(rst != 0)
    {
        ROS_ERROR("[xjrobot_inspection] 2.ERROR : create statusThread thread fail, exit");
        return rst;
    }

    rst = pthread_create(&id_task_status, NULL, pTaskStatusPro, NULL);
    if(rst != 0)
    {
        ROS_ERROR("[xjrobot_inspection] 3.ERROR : create pTaskStatusPro thread fail, exit");
        return rst;
    }

    return RST_OK;
}

void * Inspection::inspectionThread(void *)
{
    while(1)
    {
        if(currentMode != MODE_INSPECTION)
        {
            usleep(THREAD_DELAY_TIME);
            continue;
        }

        switch(inspectionStatus)
        {
            case INSPECTION_STATUS_IDEL:
                break;
            case INSPECTION_STATUS_PATH_PLANNING:
                break;
            case INSPECTION_STATUS_OPEN_DOOR:
                break;
            case INSPECTION_STATUS_GET_LOCATION:
                if(locate_state_value == LOCATE_STATE_OK)
                {
                    inspectionStatus = INSPECTION_STATUS_TAKE_OFF_PILE;
                    doorOpen.data = DOOR_STATE_OPEN;
                    doorState_.publish(doorOpen);
                }
                break;
            case INSPECTION_STATUS_TAKE_OFF_PILE:
                if(takeOffPile() == RST_OK)
                {
                    inspectionStatus = INSPECTION_STATUS_DOWNLOAD_ONE_TASK;
                }
                else
                {
                    inspectionStatus = INSPECTION_STATUS_TAKE_OFF_PILE_ERR;
                }
                break;
            case INSPECTION_STATUS_DOWNLOAD_ONE_TASK:
                if(sendOneTask(&(taskList)))
                {
                    inspectionStatus = INSPECTION_STATUS_WAIT_MOVING;
                    TaskState = TASK_START;
                    NavState = NAV_STATE_MOVING;
                }
                else
                {
                    inspectionStatus = INSPECTION_STATUS_DOWNLOAD_ONE_TASK_ERR;
                }
                break;
            case INSPECTION_STATUS_DOWNLOAD_NEXT_TASK:
                break;
            case INSPECTION_STATUS_WAIT_MOVING:
                break;
            case INSPECTION_STATUS_MOVING:
                break;
            case INSPECTION_STATUS_ONE_TASK_FINISH:
                break;
            case INSPECTION_STATUS_PATH_PLANNING_ERR:
                break;
            case INSPECTION_STATUS_DOWNLOAD_ONE_TASK_ERR:
                break;
            case INSPECTION_STATUS_MOVING_ERR:
                break;
            case INSPECTION_STATUS_TAKE_OFF_PILE_ERR:
                break;
            case INSPECTION_STATUS_GOING_TO_CHARGEROME:
                break;
            case INSPECTION_STATUS_REACHED_CHARGEROME:
                break;
            case INSPECTION_STATUS_CHARGING:
                break;
            case INSPECTION_STATUS_RETRY:
                break;
            case INSPECTION_STATUS_SPARE:
                break;
            case INSPECTION_STATUS_ERROR:
                break;
            case INSPECTION_STATUS_FAILED:
                break;
            case INSPECTION_STATUS_WAITING_DOOR_OPEN_OUTOF_TIME:
                break;
            case INSPECTION_STATUS_CHARGE_OPEN_FAILED:
                break;
            case INSPECTION_STATUS_NAVIGATION_OUTOF_TIME:
                break;
            case INSPECTION_STATUS_PILING:
                break;
            case INSPECTION_STATUS_LOCATION_ERR:
                break;
            case INSPECTION_STATUS_START_BACK_ERR:
                break;
            case INSPECTION_STATUS_GET_SAFE_AREA:
                break;
            case INSPECTION_STATUS_ARRIVE_SAFE_AREA:
                if(navMode == NAV_MODE_SPECIAL_BACK)
                {
                    one_key_to_return();
                }
                else if(navMode == NAV_MODE_SPECIAL_TASK)
                {
                    navMode = NAV_MODE_INS;
                    inspectionStatus = INSPECTION_STATUS_DOWNLOAD_ONE_TASK;
                }
                break;
            case INSPECTION_STATUS_GET_SAFE_AREA_ERR:
                break;
            default:
                inspectionStatus = INSPECTION_STATUS_IDEL;
                break;
        }
        usleep(THREAD_DELAY_TIME);
    }
}

void * Inspection::statusThread(void *)
{
  unsigned int count = 0; // log rate : 1Hz 

  while(ros::ok())
  {
    ros::param::get("W_period", W_periodValue);
    W_periodValue = W_periodValue>10?10:W_periodValue;
    W_periodValue = W_periodValue<1?1:W_periodValue;

    if(currentMode == MODE_INSPECTION)
    {
      count++;
      if(count >= W_periodValue)
      {
        count = 0;
        switch(inspectionStatus)
        {
          case INSPECTION_STATUS_IDEL:
            ROS_INFO("[xjrobot_inspection] status : idel");
            break;
          case INSPECTION_STATUS_PATH_PLANNING:
            ROS_INFO("[xjrobot_inspection] status : path planning");
            break;
          case INSPECTION_STATUS_OPEN_DOOR:
            ROS_INFO("[xjrobot_inspection] status : openning door");
            break;
          case INSPECTION_STATUS_GET_LOCATION:
            ROS_INFO("[xjrobot_inspection] status : getting location");
            break;
          case INSPECTION_STATUS_TAKE_OFF_PILE:
            ROS_INFO("[xjrobot_inspection] status : taking off pile");
            break;
          case INSPECTION_STATUS_DOWNLOAD_ONE_TASK:
            ROS_INFO("[xjrobot_inspection] status : downloading one task");
            break;
          case INSPECTION_STATUS_DOWNLOAD_NEXT_TASK:
            ROS_INFO("[xjrobot_inspection] status : downloading next task");
            break;
          case INSPECTION_STATUS_WAIT_MOVING:
            ROS_INFO("[xjrobot_inspection] status : waitting moving");
            break;
          case INSPECTION_STATUS_MOVING:
            ROS_INFO("[xjrobot_inspection] status : moving");
            break;
          case INSPECTION_STATUS_ONE_TASK_FINISH:
            ROS_INFO("[xjrobot_inspection] status : one task fanish");
            break;
          case INSPECTION_STATUS_PATH_PLANNING_ERR:
            ROS_ERROR("[xjrobot_inspection] status : path planning error!");
            break;
          case INSPECTION_STATUS_DOWNLOAD_ONE_TASK_ERR:
            ROS_ERROR("[xjrobot_inspection] status : downloading task error!");
            break;
          case INSPECTION_STATUS_MOVING_ERR:
            ROS_ERROR("[xjrobot_inspection] status : moving error!");
            break;
          case INSPECTION_STATUS_TAKE_OFF_PILE_ERR:
            ROS_ERROR("[xjrobot_inspection] status : take off pile error!");
            break;
          case INSPECTION_STATUS_GOING_TO_CHARGEROME:
            ROS_INFO("[xjrobot_inspection] status : going to charge room");   //
            break;
          case INSPECTION_STATUS_REACHED_CHARGEROME:
            ROS_INFO("[xjrobot_inspection] status : reached charge room");   //
            break;
          case INSPECTION_STATUS_CHARGING:
            ROS_INFO("[xjrobot_inspection] status : charging");   //
            break;
          case INSPECTION_STATUS_RETRY:
            ROS_INFO("[xjrobot_inspection] status : retry");   //
            break;
          case INSPECTION_STATUS_SPARE:
            ROS_INFO("[xjrobot_inspection] status : spare");   //
            break;
          case INSPECTION_STATUS_ERROR:
            ROS_ERROR("[xjrobot_inspection] status : charge error!"); //
            break;
          case INSPECTION_STATUS_FAILED:
            ROS_ERROR("[xjrobot_inspection] status : charge error!"); //
            break;
          case INSPECTION_STATUS_WAITING_DOOR_OPEN_OUTOF_TIME:
            ROS_ERROR("[xjrobot_inspection] status : waiting door open timeout!"); //
            break;
          case INSPECTION_STATUS_CHARGE_OPEN_FAILED:
            ROS_ERROR("[xjrobot_inspection] status : charge open failed!"); //
            break;
          case INSPECTION_STATUS_NAVIGATION_OUTOF_TIME:
            ROS_ERROR("[xjrobot_inspection] status : navigation timeout!"); //
            break;
          case INSPECTION_STATUS_PILING:
            ROS_INFO("[xjrobot_inspection] status : piling");   //
            break;
          case INSPECTION_STATUS_LOCATION_ERR:
            ROS_ERROR("[xjrobot_inspection] status : locate error!"); //
            break;
          case INSPECTION_STATUS_START_BACK_ERR:
            ROS_ERROR("[xjrobot_inspection] status : start back error!"); //
            break;
          case INSPECTION_STATUS_GET_SAFE_AREA:
            ROS_INFO("[xjrobot_inspection] status : going to safe area"); //
            break;
          case INSPECTION_STATUS_ARRIVE_SAFE_AREA:
            ROS_INFO("[xjrobot_inspection] status : arrived safe area"); //
            break;
          case INSPECTION_STATUS_GET_SAFE_AREA_ERR:
            ROS_ERROR("[xjrobot_inspection] status : going to safe area error!"); //
            break;
          default:
            ROS_ERROR("[xjrobot_inspection] status : status error!");
            break;
        }
      }
      status_codePub.status = inspectionStatus;
      status_code_.publish(status_codePub);
    }
    ros::spinOnce();
    ros::Duration(1.0/W_periodValue).sleep();
  }
}

void * Inspection::pTaskStatusPro(void *)
{
  while(ros::ok())
  {
    ros::param::get("Y_period", Y_periodValue);
    Y_periodValue = Y_periodValue>10?10:Y_periodValue;
    Y_periodValue = Y_periodValue<1?1:Y_periodValue;

    // publish taskStatus msg
    if(navMode == NAV_MODE_INS)
    {
      if(inspectionStatus == INSPECTION_STATUS_WAIT_MOVING || inspectionStatus == INSPECTION_STATUS_MOVING)
      {
        taskStatusPub.task_id = currentTaskId;
        taskStatusPub.task_status = currentTaskId;
        taskStatusPub.robot_angle = robot_pose[2];
        taskStatus_.publish(taskStatusPub);
      }
      else if(inspectionStatus == INSPECTION_STATUS_ONE_TASK_FINISH)
      {
        taskStatusPub.task_id = currentTaskId;
        taskStatusPub.task_status = TASK_STATUS_ARRIVE;
        taskStatusPub.robot_angle = robot_pose[2];
        taskStatus_.publish(taskStatusPub);
      }
      else if(inspectionStatus == INSPECTION_STATUS_IDEL && TaskState == TASK_START)
      {
        taskStatusPub.task_id = currentTaskId;
        taskStatusPub.task_status = TASK_STATUS_FINISH;
        taskStatusPub.robot_angle = robot_pose[2];
        taskStatus_.publish(taskStatusPub);
      }
      else if(inspectionStatus == INSPECTION_STATUS_OPEN_DOOR)
      {
        taskStatusPub.task_id = currentTaskId;
        taskStatusPub.task_status = TASK_STATUS_OPEN_DOOR;
        taskStatusPub.robot_angle = robot_pose[2];
        taskStatus_.publish(taskStatusPub);
      }
    }
    else if(navMode == NAV_MODE_BACK)
    {
      switch(inspectionStatus)
      {
        case INSPECTION_STATUS_GOING_TO_CHARGEROME:
          taskStatusPub.task_id = currentTaskId;
          taskStatusPub.task_status = currentTaskId;
          taskStatusPub.robot_angle = robot_pose[2];
          taskStatus_.publish(taskStatusPub);
          break;
        case INSPECTION_STATUS_REACHED_CHARGEROME:
          taskStatusPub.task_id = currentTaskId;
          taskStatusPub.task_status = TASK_STATUS_OPEN_DOOR;
          taskStatusPub.robot_angle = robot_pose[2];
          taskStatus_.publish(taskStatusPub);
          break;
        case INSPECTION_STATUS_PILING:
        case INSPECTION_STATUS_RETRY:
          taskStatusPub.task_id = currentTaskId;
          taskStatusPub.task_status = currentTaskId;
          taskStatusPub.robot_angle = robot_pose[2];
          taskStatus_.publish(taskStatusPub);
          break;
        case INSPECTION_STATUS_CHARGING:
          taskStatusPub.task_id = currentTaskId;
          taskStatusPub.task_status = TASK_STATUS_CLOSE_DOOR;
          taskStatusPub.robot_angle = robot_pose[2];
          taskStatus_.publish(taskStatusPub);
          break;
        case INSPECTION_STATUS_SPARE:
        case INSPECTION_STATUS_ERROR:
        case INSPECTION_STATUS_FAILED:
        case INSPECTION_STATUS_WAITING_DOOR_OPEN_OUTOF_TIME:
        case INSPECTION_STATUS_CHARGE_OPEN_FAILED:
        case INSPECTION_STATUS_NAVIGATION_OUTOF_TIME:
          break;
      }
    }
    else if(navMode == NAV_MODE_SPECIAL_BACK || navMode == NAV_MODE_SPECIAL_TASK)
    {
      taskStatusPub.task_id = currentTaskId;
      taskStatusPub.task_status = currentTaskId;
      taskStatusPub.robot_angle = robot_pose[2];
      taskStatus_.publish(taskStatusPub);
    }

    ros::spinOnce();
    ros::Duration(1.0/Y_periodValue).sleep();
  }
}

int Inspection::initPose()
{
    // 1. get charge node pose
    xjrobot_interior_msg::getPointsInfo getPointsInfo_srv;
    getPointsInfo_srv.request.type = NODE_TYPE_CHARGE;
    getPointsInfo_srv.request.number = CHARGE_NODE_NUM;
    double chargeNodePose[3];

    if(getPointsInfoClient.call(getPointsInfo_srv))
    {
        if(getPointsInfo_srv.response.points.size() == 1)
        {
            chargeNodePose[0] = getPointsInfo_srv.response.points[0].position.x;
            chargeNodePose[1] = getPointsInfo_srv.response.points[0].position.y;
            chargeNodePose[2] = getPointsInfo_srv.response.points[0].position.theta;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 4.can not get charge node pose!");
            return RST_FAIL;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 5.getPointsInfoClient.call() fail!");
        return RST_FAIL;
    }

    // 2. publish init pose
    tf::Quaternion q;
    init_pose_.pose.pose.position.x = chargeNodePose[0];
    init_pose_.pose.pose.position.y = chargeNodePose[1];
    init_pose_.pose.pose.position.z = 0.0;
    q.setRPY(0, 0, chargeNodePose[2]);
    init_pose_.pose.pose.orientation.x = q.getX();
    init_pose_.pose.pose.orientation.y = q.getY();
    init_pose_.pose.pose.orientation.z = q.getZ();
    init_pose_.pose.pose.orientation.w = q.getW();
    _init_pose_pub.publish(init_pose_);

    return RST_OK;
}

void Inspection::robot_move_front()
{
    cmd_vel_.linear.x = 0.01;
    cmd_vel_.angular.z = 0.0;
    _cmd_vel_pub.publish(cmd_vel_);
}

void Inspection::robot_move_stop()
{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    _cmd_vel_pub.publish(cmd_vel_);
}

int Inspection::takeOffPile(void)
{
    int rst = RST_OK;

#ifndef CONFIG_TAKEOFF_PILE
    return rst;
#endif

    // 1. get charge node pose
    xjrobot_interior_msg::getPointsInfo getPointsInfo_srv;
    getPointsInfo_srv.request.type = NODE_TYPE_CHARGE;
    getPointsInfo_srv.request.number = CHARGE_NODE_NUM;
    double chargeNodePose[3];
    double dis = 5000.0;
    double dis_x = 0.0;
    double dis_y = 0.0;
    double speed;

    if(getPointsInfoClient.call(getPointsInfo_srv))
    {
        if(getPointsInfo_srv.response.points.size() == 1)
        {
            chargeNodePose[0] = getPointsInfo_srv.response.points[0].position.x;
            chargeNodePose[1] = getPointsInfo_srv.response.points[0].position.y;
            chargeNodePose[2] = getPointsInfo_srv.response.points[0].position.theta;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 6.can not get charge node pose!");
            return RST_FAIL;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 7.getPointsInfoClient.call() fail!");
        return RST_FAIL;
    }

    // 2. unlock charge
    if(unlock_charge() != RST_OK)
    {
        return RST_FAIL;
    }

    // 3. robot move
    while(dis > 0.15)
    {
        if(robot_pose_state == STATE_TRUE)
        {
            if(chargeNodePose[0] > robot_pose[0])
            {
                dis_x = chargeNodePose[0] - robot_pose[0];
            }
            else
            {
                dis_x = robot_pose[0] - chargeNodePose[0];
            }
            if(chargeNodePose[1] > robot_pose[1])
            {
                dis_y = chargeNodePose[1] - robot_pose[1];
            }
            else
            {
                dis_y = robot_pose[1] - chargeNodePose[1];
            }
            dis = sqrt(dis_x*dis_x+dis_y*dis_y);
            if(dis > ROBOT_START_POSE_ERR_THRESHOLD)
            {
                ROS_ERROR("[xjrobot_inspection] 8.robot is not in charge room!");
                rst = RST_FAIL;
                break;
            }

            if(sqrt((chargeNodePose[2]-robot_pose[2])*(chargeNodePose[2]-robot_pose[2])) > ROBOT_START_THETA_ERR_THRESHOLD)
            {
                ROS_ERROR("[xjrobot_inspection] 9.robot initial pose theta is not correct!");
                rst = RST_FAIL;
                break;
            }

            if(dis <= 0.15)
            {
                break;
            }

            robot_move_front();
        }
        else
        {
            robot_move_stop();
        }
        usleep(50000);
    }

    robot_move_stop();
    usleep(50000);
    robot_move_stop();
    usleep(50000);
    robot_move_stop();
    return rst;
}

int Inspection::getCurrentPoint(double x, double y, int * point)
{
    double dis_x = 0.0;
    double dis_y = 0.0;
    double min_dis = 5000.0;
    int min_dis_point_num;
    double sqrtValue;

    // 1. get points info
    xjrobot_interior_msg::getPointsInfo srv;
    srv.request.type = NODE_TYPE_ALL;
    srv.request.number = 0;

    if(getPointsInfoClient.call(srv))
    {
        if(srv.response.points.size() >= 1)
        {
            for(int i=0; i<srv.response.points.size(); i++)
            {
                if(srv.response.points[i].position.x > x)
                {
                    dis_x = srv.response.points[i].position.x - x;
                }
                else
                {
                    dis_x = x - srv.response.points[i].position.x;
                }
                if(srv.response.points[i].position.y > y)
                {
                    dis_y = srv.response.points[i].position.y - y;
                }
                else
                {
                    dis_y = y - srv.response.points[i].position.y;
                }
                sqrtValue = sqrt(dis_x*dis_x+dis_y*dis_y);
                if(min_dis > sqrtValue)
                {
                    min_dis = sqrtValue;
                    min_dis_point_num = srv.response.points[i].number;
                }
            }
            if(min_dis < GET_POINT_INFO_THRESHOLD)
            {
                *point = min_dis_point_num;
                return RST_OK;
            }
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 10.can not get points info!");
            return RST_FAIL;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 11.getPointsInfoClient.call() fail!");
        return RST_FAIL;
    }

    return RST_FAIL;
}

int Inspection::isRobotOutDoor(double x, double y, int * robot_w)
{
    double charge_node_x = 0.0;
    double charge_node_y = 0.0;
    double sqrtValue;
    unsigned int charge_state;
    unsigned int power_state;
    int count = 0;
    double dis_x = 0.0;
    double dis_y = 0.0;

    // 1. check if robot in door
    while(count < 10)
    {
        if(base_info_state == DATA_REFRESH)
        {
            charge_state = (baseInfo.robot_state & 0x000000e0) >> 5;
            power_state  = (baseInfo.robot_state & 0x00008000)>>15;
            if(power_state==POWER_IS_RIGHT)
            {
                if(charge_state == CHARGE_STATUS_CHARGING)//||(baseInfo.power_rest>=99.9&&baseInfo.power_rest<=100))
                {
                    *robot_w = ROBOT_IN_DOOR;
                    return RST_OK;
                }
                else
                {
                    xjrobot_interior_msg::chargeOpen srv;
                    srv.request.charge_cmd=1;//open charge
                    _charge_start_client.call(srv);
                }
            }
            else
                std::cout<<"POWER_STATE is error while not use charge_state to judge Isoudoor"<<endl;
        }
        usleep(100000);
        count++;
    }


    //1.2.robot power is error and in indoor
    /*    
	count=0;
    while(count < 10)
    {

        if(baseInfo.power_rest==0&&(fabs(x)<1.5||fabs(y)<1.5))
        {
                *robot_w = ROBOT_IN_DOOR;
                return RST_OK;
        }

        usleep(100000);
        count++;
    }
	*/
	


    //2.check if robot in door using x,y
    xjrobot_interior_msg::getPointsInfo srv;
    srv.request.type = NODE_TYPE_CHARGE;
    srv.request.number = CHARGE_NODE_NUM;
    count=0;
    if(getPointsInfoClient.call(srv))
    {
        if(srv.response.points.size() >= 1)
        {
            charge_node_x = srv.response.points[0].position.x;
            charge_node_y = srv.response.points[0].position.y;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] can not get charge node location info!");
            return RST_FAIL;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] can not get points info!");
        return RST_FAIL;
    }
    while(count < 50)
    {
        if(locate_state_value == LOCATE_STATE_OK)
        {
//            if(charge_node_x > x)
//            {
//                dis_x = charge_node_x - x;
//            }
//            else
//            {
//                dis_x = x - charge_node_x;
//            }
//            if(charge_node_y > y)
//            {
//                dis_y = charge_node_y - y;
//            }
//            else
//            {
//                dis_y = y - charge_node_y;
//            }

            dis_x = x-charge_node_x ;
            dis_y = y-charge_node_y ;
            sqrtValue = sqrt(fabs(dis_x)*fabs(dis_x)+fabs(dis_y)*fabs(dis_y));
            if((dis_x >-CHARGE_HOUSE_LENGTH)&&(dis_x<CHARGE_HOUSE_LENGTH/2.0)&&(fabs(dis_y)<CHANGE_HOUSE_WIDTH/2.0))
            {
                // is indoor
                *robot_w = ROBOT_IN_DOOR;
                return RST_OK;
            }
            else
            {
                // is indoor
                break;
            }
        }

        usleep(100000);
        count++;
    }


    // 3. check if robot out door
    //xjrobot_interior_msg::getPointsInfo srv;
    srv.request.type = NODE_TYPE_CHARGE;
    srv.request.number = CHARGE_NODE_NUM;
    if(getPointsInfoClient.call(srv))
    {
        if(srv.response.points.size() >= 1)
        {
            charge_node_x = srv.response.points[0].position.x;
            charge_node_y = srv.response.points[0].position.y;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 12.can not get charge node location info!");
            return RST_FAIL;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 13.getPointsInfoClient.call() fail!");
        return RST_FAIL;
    }

    count = 0;
    while(count < 50)
    {
        if(locate_state_value == LOCATE_STATE_OK)
        {
            if(charge_node_x > x)
            {
                dis_x = charge_node_x - x;
            }
            else
            {
                dis_x = x - charge_node_x;
            }
            if(charge_node_y > y)
            {
                dis_y = charge_node_y - y;
            }
            else
            {
                dis_y = y - charge_node_y;
            }
            sqrtValue = sqrt(dis_x*dis_x+dis_y*dis_y);
            if(sqrtValue > CHARGE_HOUSE_RADIUS)
            {
                // is out door
                *robot_w = ROBOT_OUT_DOOR;
                return RST_OK;
            }
            else
            {
                // not out door
                break;
            }
        }

        usleep(100000);
        count++;
    }


    *robot_w = ROBOT_UNKNOW;
    return RST_OK;
}

void Inspection::robot_pose_timerCallback(const ros::TimerEvent& event)
{
    if(base_info_state == DATA_REFRESH)
    {
        robot_pose_state = STATE_TRUE;
    }
    else
    {
        robot_pose_state = STATE_FALSE;
    }
    base_info_state = DATA_NOREFRESH;
}

void Inspection::base_infoCallback(const xjrobot_interface_msg::base_infoConstPtr& msg)
{
    baseInfo = *msg;
    robot_pose[0] = baseInfo.pose.x;
    robot_pose[1] = baseInfo.pose.y;
    robot_pose[2] = baseInfo.pose.theta;
    base_info_state = DATA_REFRESH;
}

void Inspection::modeCallback(const xjrobot_interface_msg::mode_codeConstPtr& msg)
{
    xjrobot_interface_msg::mode_code modeCode;
    modeCode = *msg;
    lastMode = currentMode;
    currentMode = modeCode.mode;
    if(currentMode == MODE_INSPECTION && lastMode != MODE_INSPECTION)
    {
        inspectionStatus = INSPECTION_STATUS_IDEL;
    }
    if(currentMode != MODE_INSPECTION && lastMode == MODE_INSPECTION)
    {
        inspectionStatus = INSPECTION_STATUS_IDEL;
    }
}

void Inspection::nav_stateCallback(const xjrobot_interior_msg::nav_stateConstPtr& msg)
{
    xjrobot_interior_msg::nav_state navState;
    navState = *msg;

    if(navMode != NAV_MODE_INS)
    {
        return;
    }

    NavState = navState.state;
    switch(NavState)
    {
        case NAV_STATE_MOVING:
            inspectionStatus = INSPECTION_STATUS_MOVING;
            break;
        case NAV_STATE_END:
            if(inspectionStatus == INSPECTION_STATUS_MOVING)
            {
                inspectionStatus = INSPECTION_STATUS_ONE_TASK_FINISH;
            }
            break;
        case NAV_STATE_ERR:
            if(inspectionStatus == INSPECTION_STATUS_MOVING || inspectionStatus == INSPECTION_STATUS_WAIT_MOVING)
            {
                inspectionStatus = INSPECTION_STATUS_MOVING_ERR;
            }
            break;
        default:
            break;
    }
}

void Inspection::charrge_stateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    std_msgs::Int8 chargeState;
    chargeState = *msg;
    AUTOCHARGE_STATE chargeState_ = (AUTOCHARGE_STATE)(chargeState.data);

    if(navMode != NAV_MODE_BACK)
    {
        return;
    }

    switch(chargeState_)
    {
        case AUTOCHARGE_GOING_TO_CHARGEROME:
            inspectionStatus = INSPECTION_STATUS_GOING_TO_CHARGEROME;
            break;
        case AUTOCHARGE_REACHED_CHARGEROME:
            inspectionStatus = INSPECTION_STATUS_REACHED_CHARGEROME;
            break;
        case AUTOCHARGE_CHARGING:
            inspectionStatus = INSPECTION_STATUS_CHARGING;
            break;
        case AUTOCHARGE_RETRY:
            inspectionStatus = INSPECTION_STATUS_RETRY;
            break;
        case AUTOCHARGE_SPARE:
            inspectionStatus = INSPECTION_STATUS_SPARE;
            break;
        case AUTOCHARGE_ERROR:
            inspectionStatus = INSPECTION_STATUS_ERROR;
            break;
        case AUTOCHARGE_FAILED:
            inspectionStatus = INSPECTION_STATUS_FAILED;
            break;
        case AUTOCHARGE_WAITING_DOOR_OPEN_OUTOF_TIME:
            inspectionStatus = INSPECTION_STATUS_WAITING_DOOR_OPEN_OUTOF_TIME;
            break;
        case AUTOCHARGE_CHARGE_OPEN_FAILED:
            inspectionStatus = INSPECTION_STATUS_CHARGE_OPEN_FAILED;
            break;
        case AUTOCHARGE_NAVIGATION_OUTOF_TIME:
            inspectionStatus = INSPECTION_STATUS_NAVIGATION_OUTOF_TIME;
            break;
        case AUTOCHARGE_PILING:
            inspectionStatus = INSPECTION_STATUS_PILING;
            break;
        default:
            ROS_ERROR("[xjrobot_inspection] autocharge state error!");
            break;
    }
}

bool Inspection::isTaskNode(int node, int startNode, vector<int> taskList, int taskVisit[])
{
    if(node == startNode)
    {
        return false;
    }

    for(int x=0; x<taskList.size(); x++)
    {
        if(node == taskList[x])
        {
            if(taskVisit[x] == VISIT_NO)
            {
                taskVisit[x] = VISIT_YES;
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

bool Inspection::isTaskNode(int node, vector<int> taskList)
{
    for(int i=0; i<taskList.size(); i++)
    {
        if(node == taskList[i])
        {
            return true;
        }
    }

    return false;
}

bool Inspection::sendOneTask(Task_List * taskL)
{
    xjrobot_interior_msg::nav_task srv;

    if(taskL->task_index >= taskL->task_num)
    {
        return false;
    }

    srv.request.nav_path.clear();
		ROS_INFO("[xjrobot_inspection] send one task, node num : %d", taskL->task_list[taskL->task_index].path_num);
    for(int x=0; x<taskL->task_list[taskL->task_index].path_num; x++)
    {
				ROS_INFO("[xjrobot_inspection] %d", taskL->task_list[taskL->task_index].path_node_list[x]);
        srv.request.nav_path.push_back(taskL->task_list[taskL->task_index].path_node_list[x]);
    }

    if(nav_taskClient.call(srv))
    {
        if(srv.response.rst.rst == RST_OK)
        {
            ROS_INFO("[xjrobot_inspection] 1.Success to call service nav_task");
            taskL->task_index++;
            currentTaskId = srv.request.nav_path[srv.request.nav_path.size()-1];
            return true;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 14.Failed to call service nav_task");
            return false;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 15.nav_taskClient.call() fail!");
        return false;
    }
}

bool Inspection::route_routeOrderCallback(xjrobot_interface_msg::route_routeOrder::Request  &req,
                                                        xjrobot_interface_msg::route_routeOrder::Response &res)
{
    int rst;
    vector <geometry_msgs::PoseStamped> poses;
    xjrobot_interior_msg::special_nav_status special_nav_status_;
    int clearBackFlag;
    int robot_where;

    if(currentMode != MODE_INSPECTION)
    {
        ROS_INFO("[xjrobot_inspection] 2.Robot is not at MODE_INSPECTION mode");
        res.rst.rst = RST_FAIL;
        return true;
    }

    if(inspectionStatus == INSPECTION_STATUS_PATH_PLANNING
        || inspectionStatus == INSPECTION_STATUS_OPEN_DOOR
        || inspectionStatus == INSPECTION_STATUS_GET_LOCATION
        || inspectionStatus == INSPECTION_STATUS_TAKE_OFF_PILE
        || inspectionStatus == INSPECTION_STATUS_DOWNLOAD_ONE_TASK
        || inspectionStatus == INSPECTION_STATUS_DOWNLOAD_NEXT_TASK
        || inspectionStatus == INSPECTION_STATUS_WAIT_MOVING
        || inspectionStatus == INSPECTION_STATUS_MOVING
        || inspectionStatus == INSPECTION_STATUS_ONE_TASK_FINISH
        || inspectionStatus == INSPECTION_STATUS_GOING_TO_CHARGEROME
        || inspectionStatus == INSPECTION_STATUS_REACHED_CHARGEROME
        || inspectionStatus == INSPECTION_STATUS_PILING
        || inspectionStatus == INSPECTION_STATUS_RETRY)
    {
        ROS_INFO("[xjrobot_inspection] 3.Robot is busy, The operation cannot be performed");
        res.rst.rst = RST_BUSY;
        return true;
    }

    navMode = NAV_MODE_INS;
    inspectionStatus = INSPECTION_STATUS_PATH_PLANNING;

    rst = isRobotOutDoor(robot_pose[0], robot_pose[1], &robot_where);
    if(rst != RST_OK)
    {
        inspectionStatus = INSPECTION_STATUS_LOCATION_ERR;
        ROS_ERROR("[xjrobot_inspection] 16.can not get current location!");
        res.rst.rst = RST_FAIL;
        return true;
    }

    /******** turn back ********/
    if(req.task_node.size() == 2 && req.task_node[0] == START_NODE_NUM && req.task_node[1] == CHARGE_NODE_NUM)
    {
        navMode = NAV_MODE_SPECIAL_BACK;
        if(robot_where != ROBOT_OUT_DOOR)
        {
            inspectionStatus = INSPECTION_STATUS_LOCATION_ERR;
            ROS_ERROR("[xjrobot_inspection] 17.robot is not out door!");
            res.rst.rst = RST_FAIL;
            return true;
        }

        special_nav_status_.request.moveFlag = 0;
        if(special_nav_statusClient.call(special_nav_status_))
        {
            if(special_nav_status_.response.rst.rst == RST_OK)
            {
                xjrobot_interior_msg::path_planning srv_back;
                srv_back.request.sP = special_nav_status_.response.disNode;
                srv_back.request.eP = CHARGE_NODE_NUM;

                // push_back task node list
                srv_back.request.pointsSeq.clear();

                if(path_planningClient.call(srv_back))
                {
                    poses.clear();
                    poses = srv_back.response.roadPath.poses;
                    if(poses.size() < 2)
                    {
                        inspectionStatus = INSPECTION_STATUS_PATH_PLANNING_ERR;
                        ROS_ERROR("[xjrobot_inspection] 18.Failed to call service path_planning");
                        res.rst.rst = RST_FAIL;
                        return true;
                    }
                    res.sequence_node.clear();
                    for(int x=0; x<poses.size(); x++)
                    {
                        res.sequence_node.push_back((unsigned int)(poses[x].pose.position.z));
                    }
                    special_nav_status_.request.moveFlag = 1;
                    if(special_nav_statusClient.call(special_nav_status_))
                    {
                        if(special_nav_status_.response.rst.rst == RST_OK)
                        {
                            usleep(1000000);
                            lineNavState = BEGAIN;
                            inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA;
                            currentTaskId = START_NODE_NUM;
                            // unlock charge
                            if(unlock_charge() != RST_OK)
                            {
                                ROS_ERROR("[xjrobot_inspection] 19.unlock_charge fail!");
                            }
                            res.rst.rst = RST_OK;
                            return true;
                        }
                        else
                        {
                            inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                            ROS_ERROR("[xjrobot_inspection] 20.can not get to safety area!");
                            res.rst.rst = RST_FAIL;
                            return true;
                        }
                    }
                    else
                    {
                        inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                        ROS_ERROR("[xjrobot_inspection] 21.special_nav_statusClient.call() fail!");
                        res.rst.rst = RST_FAIL;
                        return true;
                    }
                }
                else
                {
                    inspectionStatus = INSPECTION_STATUS_PATH_PLANNING_ERR;
                    ROS_ERROR("[xjrobot_inspection] 22.path_planningClient.call() fail!");
                    res.rst.rst = ROBOT_ERROR;
                    return true;
                }
            }
            else
            {
                inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                ROS_ERROR("[xjrobot_inspection] 23.can not get to safety area!");
                res.rst.rst = RST_FAIL;
                return true;
            }
        }
        else
        {
            inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
            ROS_ERROR("[xjrobot_inspection] 24.special_nav_statusClient.call() fail!");
            res.rst.rst = RST_FAIL;
            return true;
        }
    }

    /******** inspection ********/
    xjrobot_interior_msg::path_planning srv;
    //std::cout<<"[xjrobot_inspection]robot_where :"<<robot_where<<std::endl;

    if(robot_where == ROBOT_IN_DOOR)
    {
        srv.request.sP = START_NODE_NUM; // start node is 1
        srv.request.eP = START_NODE_NUM; // end node is 1
    }
    else if(robot_where == ROBOT_OUT_DOOR)
    {
        special_nav_status_.request.moveFlag = 0;
        if(special_nav_statusClient.call(special_nav_status_))
        {
            if(special_nav_status_.response.rst.rst == RST_OK)
            {
                srv.request.sP = special_nav_status_.response.disNode;
                srv.request.eP = ANY_NODE_TASK;
            }
            else
            {
                inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                ROS_ERROR("[xjrobot_inspection] 25.can not get to safety area!");
                res.rst.rst = RST_FAIL;
                return true;
            }
        }
        else
        {
            inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
            ROS_ERROR("[xjrobot_inspection] 26.special_nav_statusClient.call() fail!");
            res.rst.rst = RST_FAIL;
            return true;
        }
    }
    else
    {
        inspectionStatus = INSPECTION_STATUS_LOCATION_ERR;
        ROS_ERROR("[xjrobot_inspection] 27.robot location is unknow!");
        res.rst.rst = RST_FAIL;
        return true;
    }
        

    // push_back task node list
    srv.request.pointsSeq.clear();
    for(int x=0; x<req.task_node.size(); x++)
    {
        srv.request.pointsSeq.push_back(req.task_node[x]);
    }

    if(path_planningClient.call(srv))
    {
        ROS_INFO("[xjrobot_inspection] 4.Success to call service path_planning");
        poses.clear();
        poses = srv.response.roadPath.poses;

        if(poses.size() < 2)
        {
            inspectionStatus = INSPECTION_STATUS_PATH_PLANNING_ERR;
            res.rst.rst = RST_FAIL;
            return true;
        }

        res.sequence_node.clear();
        for(int x=0; x<poses.size(); x++)
        {
            res.sequence_node.push_back((unsigned int)(poses[x].pose.position.z));

            // clear back way
            for(int y=0; y<srv.request.pointsSeq.size(); y++)
            {
                if(srv.request.pointsSeq[y] == (unsigned int)(poses[x].pose.position.z))
                {
                    srv.request.pointsSeq[y] = 0;
                }
            }

            clearBackFlag = 0;
            for(int z=0; z<srv.request.pointsSeq.size(); z++)
            {
                clearBackFlag += srv.request.pointsSeq[z];
            }

            if(clearBackFlag == 0)
            {
                break;
            }
        }

        // init task list
        memset(&(taskList), 0x00, sizeof(Task_List));
        int taskVisit[MAX_NODE_NUM];
        memset(taskVisit, 0x00, sizeof(taskVisit));
        // if the path planning first node is a task node
        if(isTaskNode(res.sequence_node[0], req.task_node))
        {
            taskList.task_list[taskList.task_num].path_node_list[0] = res.sequence_node[0];
            taskList.task_list[taskList.task_num].path_num++;
            taskList.task_num++;
        }
        for(int x=0; x<res.sequence_node.size(); x++)
        {
            taskList.task_list[taskList.task_num].path_node_list[taskList.task_list[taskList.task_num].path_num] = res.sequence_node[x];
            taskList.task_list[taskList.task_num].path_num++;
            if(isTaskNode(res.sequence_node[x], res.sequence_node[0], req.task_node, taskVisit))
            {
                taskList.task_num++;
            }
        }
        taskList.task_index = 0;
				ROS_INFO("[xjrobot_inspection] get one task list, task num : %d", taskList.task_num);
				for(int t=0; t<taskList.task_num; t++)
				{
					ROS_INFO("[xjrobot_inspection] task index : %d", t);
					for(int t1=0; t1<taskList.task_list[t].path_num; t1++)
					{
						ROS_INFO("[xjrobot_inspection] %d", taskList.task_list[t].path_node_list[t1]);
					}
				}
        // init task list end

        if(robot_where == ROBOT_IN_DOOR)
        {
            navMode = NAV_MODE_INS;
            inspectionStatus = INSPECTION_STATUS_OPEN_DOOR;
            currentTaskId = CHARGE_NODE_NUM;
        }
        else if(robot_where == ROBOT_OUT_DOOR)
        {
            special_nav_status_.request.moveFlag = 1;
            if(special_nav_statusClient.call(special_nav_status_))
            {
                if(special_nav_status_.response.rst.rst == RST_OK)
                {
                    lineNavState = BEGAIN;
                    navMode = NAV_MODE_SPECIAL_TASK;
                    inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA;
                    currentTaskId = taskList.task_list[taskList.task_index].path_node_list[taskList.task_list[taskList.task_index].path_num-1];
                }
                else
                {
                    inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                    ROS_ERROR("[xjrobot_inspection] 28.can not get to safety area!");
                    res.rst.rst = RST_FAIL;
                    return true;
                }
            }
            else
            {
                inspectionStatus = INSPECTION_STATUS_GET_SAFE_AREA_ERR;
                ROS_ERROR("[xjrobot_inspection] 29.special_nav_statusClient.call() fail!");
                res.rst.rst = RST_FAIL;
                return true;
            }
        }

        res.rst.rst = RST_OK;
    }
    else
    {
        ROS_ERROR("[xjrobot_inspection] 30.path_planningClient.call() fail!");
        inspectionStatus = INSPECTION_STATUS_PATH_PLANNING_ERR;
        res.rst.rst = RST_FAIL;
        return true;
    }

    res.rst.rst = RST_OK;
    return true;
}

int Inspection::one_key_to_return()
{
    int rst;

    navMode = NAV_MODE_BACK;

    // 1. get current location point
    if(robot_pose_state == STATE_TRUE)
    {
        rst = getCurrentPoint(robot_pose[0], robot_pose[1], &currentPoint);
        if(rst != RST_OK)
        {
            inspectionStatus = INSPECTION_STATUS_LOCATION_ERR;
            ROS_ERROR("[xjrobot_inspection] 31.can not get current location point!");
            return RST_FAIL;
        }
    }
    else
    {
        inspectionStatus = INSPECTION_STATUS_LOCATION_ERR;
        ROS_ERROR("[xjrobot_inspection] 32.can not get current location point!");
        return RST_FAIL;
    }

    ROS_INFO("[xjrobot_inspection] 5.Robot current point is : %d", currentPoint);
    xjrobot_interior_msg::autoCharge_chargeOrder srv;
    srv.request.Current_point = currentPoint;
    srv.request.charge_num = START_NODE_NUM;
    if(autoCharge_chargeOrderClient.call(srv))
    {
        if(srv.response.rst.rst == RST_OK)
        {
            inspectionStatus = INSPECTION_STATUS_WAIT_MOVING;
            TaskState = TASK_START;
            //NavState = NAV_STATE_MOVING;
            currentTaskId = START_NODE_NUM;
            return RST_OK;
        }
        else
        {
            inspectionStatus = INSPECTION_STATUS_START_BACK_ERR;
            ROS_ERROR("[xjrobot_inspection] 33.Failed to call service autoCharge_chargeOrder");
            return ROBOT_ERROR;
        }
    }
    else
    {
        inspectionStatus = INSPECTION_STATUS_START_BACK_ERR;
        ROS_ERROR("[xjrobot_inspection] 34.autoCharge_chargeOrderClient.call() fail!");
        return ROBOT_ERROR;
    }
}

bool Inspection::route_nextOrderCallback(xjrobot_interface_msg::route_nextOrder::Request  &req,
                                                        xjrobot_interface_msg::route_nextOrder::Response &res)
{
    if(currentMode != MODE_INSPECTION)
    {
        ROS_INFO("[xjrobot_inspection] 6.Robot is not at MODE_INSPECTION mode");
        res.rst.rst = RST_FAIL;
        return true;
    }

    res.rst.rst = RST_OK;
    if(navMode == NAV_MODE_INS)
    {
        if(inspectionStatus == INSPECTION_STATUS_ONE_TASK_FINISH)
        {
            if(taskList.task_index >= taskList.task_num)
            {
                inspectionStatus = INSPECTION_STATUS_IDEL;
                res.rst.rst = RST_OK;
            }
            else
            {
                inspectionStatus = INSPECTION_STATUS_DOWNLOAD_NEXT_TASK;
                if(sendOneTask(&(taskList)))
                {
                    inspectionStatus = INSPECTION_STATUS_WAIT_MOVING;
                    NavState = NAV_STATE_MOVING;
                    res.rst.rst = RST_OK;
                }
                else
                {
                    inspectionStatus = INSPECTION_STATUS_DOWNLOAD_ONE_TASK_ERR;
                    res.rst.rst = RST_FAIL;
                }
            }
        }
        else if(inspectionStatus == INSPECTION_STATUS_OPEN_DOOR)
        {
            if(initPose() != RST_OK)
            {
                inspectionStatus = INSPECTION_STATUS_TAKE_OFF_PILE_ERR;
                res.rst.rst = RST_OK;
                return true;
            }

            inspectionStatus = INSPECTION_STATUS_GET_LOCATION;
            res.rst.rst = RST_OK;
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 35.Task is not completed");
            res.rst.rst = RST_FAIL;
        }
    }
    else if(navMode == NAV_MODE_BACK)
    {
        if(inspectionStatus == INSPECTION_STATUS_REACHED_CHARGEROME)
        {
            currentTaskId = CHARGE_NODE_NUM;
            doorStatePub.data = DOOR_STATE_OPEN;
            doorState_.publish(doorStatePub);
        }
        else if(inspectionStatus == INSPECTION_STATUS_CHARGING)
        {
            TaskState = TASK_END;
            navMode = NAV_MODE_INS;
            usleep(50000);
            inspectionStatus = INSPECTION_STATUS_IDEL;
            doorStatePub.data = DOOR_STATE_CLOSE;
            doorState_.publish(doorStatePub);
        }
    }
    return true;
}

int Inspection::unlock_charge()
{
    xjrobot_interior_msg::chargeOpen chargeOpenClient_srv;
    chargeOpenClient_srv.request.charge_cmd = CHARGE_UNLOCK;
    int chargeOpenErrCnt = 0;

    for(int x=0; x<5; x++)
    {
        if(chargeOpenClient.call(chargeOpenClient_srv))
        {
            if(chargeOpenClient_srv.response.rst.rst == RST_OK)
            {
                break;
            }
            else
            {
                ROS_ERROR("[xjrobot_inspection] 36.unlock charge error!");
                chargeOpenErrCnt++;
            }
        }
        else
        {
            ROS_ERROR("[xjrobot_inspection] 37.chargeOpenClient.call() fail!");
            chargeOpenErrCnt++;
        }

        if(chargeOpenErrCnt >= 3)
        {
            ROS_ERROR("[xjrobot_inspection] 38.can not unlock charge!");
            return RST_FAIL;
        }
    }

    usleep(500000);
    return RST_OK;
}

void Inspection::locate_stateCallback(const xjrobot_interior_msg::locate_stateConstPtr& msg)
{
    locate_state_ = *msg;
    locate_state_value = locate_state_.state;
}

void Inspection::lineNavStateCallback(const xjrobot_interior_msg::flagConstPtr& msg)
{
    xjrobot_interior_msg::flag flagT;
    flagT = *msg;
    lineNavState = flagT.flag;

    if(inspectionStatus == INSPECTION_STATUS_GET_SAFE_AREA)
    {
        if(lineNavState == REACHED)
        {
            inspectionStatus = INSPECTION_STATUS_ARRIVE_SAFE_AREA;
            lineNavState = BEGAIN;
        }
    }
}
