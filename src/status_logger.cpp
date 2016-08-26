///////////////////////////////////////////////////////////////////////////////
//      Title     : Status logger
//      Project   : ROSSTEP
//      Created   : 7/15/2015
//      Author    : Adam Allevato
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "status_panel/status_logger.h"

StatusLogger::StatusLogger(std::string topic_, std::string defaultColor_) :
  defaultColor(defaultColor_),
  topic(topic_),
  nh()
{
  publisher = nh.advertise<std_msgs::String>(topic_, MESSAGE_QUEUE_SIZE, true); //latch
  ROS_INFO("Status Logger initialized.");
}

void StatusLogger::setDefaultColor(const std::string& defaultColor_)
{
  defaultColor = defaultColor_;
}

void StatusLogger::setTopic(std::string topicName)
{
  topicName = ros::names::clean(topicName);
  if(topicName == topic) return;

  //we need to validate.
  std::string errorStr;
  try {
    ros::names::validate(topicName, errorStr);
  } catch(const ros::InvalidNameException& e) {
    ROS_ERROR_STREAM("Tried to pass an invalid name to StatusLogger: " << errorStr << std::endl << "Will not change publishing topic.");
    return;
  }

  topic = topicName;
  publisher.shutdown();
  publisher = nh.advertise<std_msgs::String>(topic, MESSAGE_QUEUE_SIZE);
}

void StatusLogger::log(std::string message, std::string color = "") {
  std::string htmlMessage = "";
  if(color != "") { //color override
    htmlMessage += "<span style='color:" + color + ";'>" + message + "</span>";
  } else if(defaultColor != "") { //fall back to default color
    htmlMessage += "<span style='color:" + defaultColor + ";'>" + message + "</span>";
  } else { //no override, default not set
    htmlMessage += message;
  }
  logHTML(htmlMessage);
}

void StatusLogger::logHTML(std::string htmlMessage) {
  //build the stupid ROS message
  msg.data = htmlMessage;

  //publish
  publisher.publish(msg);

  //force a spin so that the message gets published
  ros::spinOnce();
}
