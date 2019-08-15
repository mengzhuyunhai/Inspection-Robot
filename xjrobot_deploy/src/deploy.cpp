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

/* Author: Zhang Chunyang */

#include "deploy.h"
#include "CsvProcess.h"

int Deploy::currentMode;
char Deploy::deployStatus;
double Deploy::W_periodValue;
ros::Publisher Deploy::status_code_;
xjrobot_interface_msg::status_code Deploy::status_codePub;

int Deploy::init(int argc, char **argv)
{
    int rst = RST_OK;

    lastMode = MODE_IDLE;
    currentMode = MODE_IDLE;
    NavState = NAV_STATE_END;
    deployStatus = DEPLOY_STATUS_IDEL;

    nav_taskClient = n.serviceClient<xjrobot_interior_msg::nav_task>("nav_task");
    status_code_ = n.advertise<xjrobot_interface_msg::status_code>("status", 10);
    modeSub = n.subscribe("/mode", 10, &Deploy::modeCallback, this);
    nav_stateSub = n.subscribe("/nav_state", 10, &Deploy::nav_stateCallback, this);
    deploy_input_fileService = n.advertiseService("deploy_input_file", &Deploy::deploy_inputFileCallback, this);
    deploy_travelService = n.advertiseService("deploy_travel", &Deploy::deploy_travelCallback, this);
    path_planningClient = n.serviceClient<xjrobot_interior_msg::path_planning>("path_planning");

    rst = pthread_create(&deployId, NULL, deployThread, NULL);
    if(rst != 0)
    {
        ROS_ERROR("[xjrobot_deploy] ERROR : [%s][%s] init deploy thread fail, exit", __FILE__, __FUNCTION__);
        return RST_FAIL;
    }

    rst = pthread_create(&statusId, NULL, statusThread, NULL);
    if(rst != 0)
    {
        ROS_ERROR("[xjrobot_deploy] ERROR : [%s][%s] init status thread fail, exit", __FILE__, __FUNCTION__);
        return RST_FAIL;
    }

    return RST_OK;
}

void * Deploy::deployThread(void *)
{
    while(1)
    {
        if(currentMode != MODE_DEPLOY)
        {
            usleep(THREAD_DELAY_TIME);
            continue;
        }

        switch(deployStatus)
        {
            case DEPLOY_STATUS_IDEL:
                break;
            case DEPLOY_STATUS_CREATE_NODES:
                break;
            case DEPLOY_STATUS_INPUT_FILE:
                break;
            case DEPLOY_STATUS_PATH_PLANNING:
                break;
            case DEPLOY_STATUS_DOWNLOAD_PATH:
                break;
            case DEPLOY_STATUS_WAIT_MOVING:
                break;
            case DEPLOY_STATUS_MOVING:
                break;
            default:
                deployStatus = DEPLOY_STATUS_IDEL;
                break;
        }
        usleep(THREAD_DELAY_TIME);
    }
}

void * Deploy::statusThread(void *)
{
  unsigned int count = 0; // log rate : 1Hz 

  while(ros::ok())
  {
    ros::param::get("W_period", W_periodValue);
    W_periodValue = W_periodValue>10?10:W_periodValue;
    W_periodValue = W_periodValue<1?1:W_periodValue;

    if(currentMode == MODE_DEPLOY)
    {
      count++;
      if(count >= W_periodValue)
      {
        count = 0;
        switch(deployStatus)
        {
          case DEPLOY_STATUS_IDEL:
            ROS_INFO("[xjrobot_deploy] status : idel");
            break;
          case DEPLOY_STATUS_CREATE_NODES:
            ROS_INFO("[xjrobot_deploy] status : creating nodes");
            break;
          case DEPLOY_STATUS_INPUT_FILE:
            ROS_INFO("[xjrobot_deploy] status : inputting file");
            break;
          case DEPLOY_STATUS_PATH_PLANNING:
            //ROS_INFO("[xjrobot_deploy] status : path planning");
            break;
          case DEPLOY_STATUS_DOWNLOAD_PATH:
            ROS_INFO("[xjrobot_deploy] status : downloading path");
            break;
          case DEPLOY_STATUS_WAIT_MOVING:
            ROS_INFO("[xjrobot_deploy] status : waiting moving");
            break;
          case DEPLOY_STATUS_MOVING:
            ROS_INFO("[xjrobot_deploy] status : moving");
            break;
          case DEPLOY_STATUS_CREATE_NODES_ERR:
            ROS_ERROR("[xjrobot_deploy] status : creating nodes error!");
            break;
          case DEPLOY_STATUS_INPUT_FILE_ERR:
            ROS_ERROR("[xjrobot_deploy] status : inputting file error!");
            break;
          case DEPLOY_STATUS_PATH_PLANNING_ERR:
            ROS_ERROR("[xjrobot_deploy] status : path planning error!");
            break;
          case DEPLOY_STATUS_DOWNLOAD_PATH_ERR:
            ROS_ERROR("[xjrobot_deploy] status : downloading path error!");
            break;
          case DEPLOY_STATUS_MOVING_ERR:
            ROS_ERROR("[xjrobot_deploy] status : moving error!");
            break;
          default:
            ROS_ERROR("[xjrobot_deploy] status : status error!");
            break;
        }
      }
      status_codePub.status = deployStatus;
      status_code_.publish(status_codePub);
    }
    ros::spinOnce();
    ros::Duration(1.0/W_periodValue).sleep();
  }
}

int Deploy::doSystem(const char * cmd)
{
    int result;
    result = system(cmd);
    if(result>=0 && WIFEXITED(result) && WEXITSTATUS(result)==0)
    {
        return RST_OK;
    }
    else
    {
        return RST_FAIL;
    }
}

void Deploy::modeCallback(const xjrobot_interface_msg::mode_codeConstPtr& msg)
{
    xjrobot_interface_msg::mode_code modeCode;
    modeCode = *msg;
    lastMode = currentMode;
    currentMode = modeCode.mode;
    if(currentMode == MODE_DEPLOY && lastMode != MODE_DEPLOY)
    {
        deployStatus = DEPLOY_STATUS_IDEL;
    }
}

void Deploy::nav_stateCallback(const xjrobot_interior_msg::nav_stateConstPtr& msg)
{
    xjrobot_interior_msg::nav_state navState;
    navState = *msg;
    NavState = navState.state;

    switch(NavState)
    {
        case NAV_STATE_MOVING:
            deployStatus = DEPLOY_STATUS_MOVING;
            break;
        case NAV_STATE_END:
            if(deployStatus == DEPLOY_STATUS_MOVING)
            {
                deployStatus = DEPLOY_STATUS_IDEL;
            }
            break;
        case NAV_STATE_ERR:
            if(deployStatus == DEPLOY_STATUS_MOVING || deployStatus == DEPLOY_STATUS_WAIT_MOVING)
            {
                deployStatus = DEPLOY_STATUS_MOVING_ERR;
            }
            break;
        default:
            break;
    }
}

bool Deploy::deploy_inputFileCallback(xjrobot_interface_msg::deploy_input_file::Request  &req,
                                                        xjrobot_interface_msg::deploy_input_file::Response &res)
{
    int rst = RST_FAIL;
    int totalLen = 0;
    string deployFileName;
    Csv_Mapmanage_Data mapData;
    char cmd[DIR_MAX_LEN];
    CsvProcess csv;
    int index;

    ROS_INFO("[xjrobot_deploy] deploy_input.name = %s", req.name.c_str());

    if(currentMode != MODE_DEPLOY)
    {
        ROS_INFO("[xjrobot_deploy] Robot is not at MODE_DEPLOY mode");
        res.rst.rst = RST_FAIL;
        return true;
    }

    if(NavState == NAV_STATE_MOVING)
    {
        ROS_INFO("[xjrobot_deploy] Robot is moving, The operation cannot be performed");
        res.rst.rst = RST_FAIL;
        return true;
    }

    deployStatus = DEPLOY_STATUS_INPUT_FILE;

    // copy to mapmanage
    rst = csv.readCsvMapmanage(&mapData);
    if(rst != RST_OK)
    {
        ROS_ERROR("[xjrobot_deploy] open mapmanage.csv file fail");
        res.rst.rst = rst;
        deployStatus = DEPLOY_STATUS_INPUT_FILE_ERR;
        return true;
    }

    if(strlen(mapData.mapusing) <= 0 || strcmp(mapData.mapusing, "null") == 0 || strcmp(mapData.mapusing, "NULL") == 0)
    {
        ROS_ERROR("[xjrobot_deploy] no map using");
        res.rst.rst = RST_FAIL;
        deployStatus = DEPLOY_STATUS_INPUT_FILE_ERR;
        return true;
    }

    deployFileName = DEPLOY_INPUT_PATH + req.name;
    memset(cmd, 0x00, sizeof(cmd));
    sprintf(cmd, "rm -rf %s%s/%s%s", MAPMANAGE_DIR, mapData.mapusing, mapData.mapusing, ".csv");
    system(cmd);
    memset(cmd, 0x00, sizeof(cmd));
    sprintf(cmd, "cp %s %s%s/%s%s", deployFileName.c_str(), MAPMANAGE_DIR, mapData.mapusing, mapData.mapusing, ".csv");
    system(cmd);

    memset(cmd, 0x00, sizeof(cmd));
    sprintf(cmd, "%s%s", mapData.mapusing, ".csv");
    for(index=0; index<mapData.map_num; index++)
    {
        if(strcmp(mapData.map_list[index].map_name, mapData.mapusing) == 0)
        {
            memset(mapData.map_list[index].node_name, 0x00, sizeof(mapData.map_list[index].node_name));
            memcpy(mapData.map_list[index].node_name, cmd, strlen(cmd));
            break;
        }
    }

    if(index > mapData.map_num)
    {
        ROS_ERROR("[xjrobot_deploy] mapmanage.csv file error");
        res.rst.rst = RST_FAIL;
        deployStatus = DEPLOY_STATUS_INPUT_FILE_ERR;
        return true;
    }

    rst = csv.writeCsvMapmanage(&mapData);
    if(rst != RST_OK)
    {
        ROS_ERROR("[xjrobot_deploy] open mapmanage.csv file fail");
        res.rst.rst = rst;
        deployStatus = DEPLOY_STATUS_INPUT_FILE_ERR;
        return true;
    }

    // set para
    csv.ParaRefreshMap(&mapData);

    deployStatus = DEPLOY_STATUS_IDEL;
    res.rst.rst = RST_OK;
    return true;
}

bool Deploy::deploy_travelCallback(xjrobot_interface_msg::deploy_travel::Request  &req,
                                                xjrobot_interface_msg::deploy_travel::Response &res)
{
    int rst = RST_OK;
    xjrobot_interior_msg::nav_task srv;

    if(currentMode != MODE_DEPLOY)
    {
        ROS_INFO("[xjrobot_deploy] Robot is not at MODE_DEPLOY mode");
        res.rst.rst = RST_FAIL;
        return true;
    }

    if(NavState == NAV_STATE_MOVING)
    {
        ROS_INFO("[xjrobot_deploy] Robot is moving, The operation cannot be performed");
        res.rst.rst = RST_FAIL;
        return true;
    }

    deployStatus = DEPLOY_STATUS_DOWNLOAD_PATH;

    if(RoutePath.size() <= 0)
    {
        ROS_INFO("[xjrobot_deploy] The path is empty!");
        res.rst.rst = RST_FAIL;
        deployStatus = DEPLOY_STATUS_DOWNLOAD_PATH_ERR;
        return true;
    }

    srv.request.nav_path.clear();
    for(int x=0; x<RoutePath.size(); x++)
    {
        srv.request.nav_path.push_back(RoutePath[x]);
    }

    if(nav_taskClient.call(srv))
    {
        if(srv.response.rst.rst == RST_OK)
        {
            ROS_INFO("[xjrobot_deploy] Success to call service nav_task");
            res.rst.rst = RST_OK;
            return true;
        }
        else
        {
            ROS_ERROR("[xjrobot_deploy] Failed to call service nav_task");
            deployStatus = DEPLOY_STATUS_DOWNLOAD_PATH_ERR;
            res.rst.rst = RST_FAIL;
            return true;
        }
    }
    else
    {
        ROS_ERROR("[xjrobot_deploy] Failed to call service nav_task");
        deployStatus = DEPLOY_STATUS_DOWNLOAD_PATH_ERR;
        res.rst.rst = RST_FAIL;
        return true;
    }

    deployStatus = DEPLOY_STATUS_WAIT_MOVING;
    return true;
}
