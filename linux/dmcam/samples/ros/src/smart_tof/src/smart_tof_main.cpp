/*****************************************************************//**
 *       @file  smart_tof_main.cpp
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  6/13/2017 
 *    Revision  $Id$
 *     Company  Data Miracle, Shanghai
 *   Copyright  (C) 2017 Data Miracle Intelligent Technologies
 *    
 *    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *    PARTICULAR PURPOSE.
 *
 * *******************************************************************/
/*
 *
 *this is the main node that subscribes parameters, gets data from 
 *dmcam and publishs it.
 *
 */

#include "tofhandle.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "smart_tof");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  TofHandle th(&n);

  int cnt=0;
  if(th.get_tof_state())
      ROS_INFO("[SMART TOF]RUNNING...");

  while(ros::ok() && th.get_tof_state()){
      th.pub_all();
      if(th.get_test_mode()){
          ROS_INFO("THE %d time...",cnt++);
      }
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

