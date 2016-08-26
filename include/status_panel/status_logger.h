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

#ifndef STATUS_LOGGER_H
#define STATUS_LOGGER_H

#include <string>
#include <map>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * A helper C++ class for publishing messages designed for the StatusPanel RViz plugin. Handles HTML
 * colors and deals with the ROS setup/teardown.
 */
class StatusLogger
{
public:
  /**
   * Create a new StatusLogger that will publish to the supplied topic. The default text color is black.
   * @param topic the ROS topic to publish to. This can be changed in the future using setTopic().
   * @param defaultColor by default, all status messages posted to the StatusPanel will have this color.
   *                     this can be overridden on a message-by-message basis, or can be changed later
   *                     using setDefaultColor().
   * See the documentation for setDefaultColor() for more information on possible color codes.
   */
  StatusLogger(std::string topic = "/status", std::string defaultColor = "");

  /**
   * @brief Set the default color to use for messages.
   * The supplied color will be inserted into CSS styling for the posted messages. In general, you should usually use one of two types of color definition:
   *   - Hex values, such as #000 (black) or #a9144c (crimson). See 
   *   - Named colors as defined by CSS. See http://www.w3schools.com/cssref/css_colors_group.asp.
   * This is not a complete list. Any CSS-value color will work. See http://www.w3schools.com/cssref/css_colors_legal.asp for all options.
   *
   * If the color is not valid, the default will be the RViz/Qt default (most likely black).
   *
   * @param defaultColor the new default color to set.
   */ 
  void setDefaultColor(const std::string& defaultColor);

  /**
   * Set the topic to publish status messages to.
   * This will result in a publisher restart if the name changes.
   * @param topicName the topic to publish status messages to.
   */
  void setTopic(std::string topicName);

  /**
   * Logs a status message that StatusPanel can understand, in the given color.
   * This is a fancy alias for logHTML() that allows you to set the text color on-the-fly.
   * @param message the message text to publish.
   * @param color an optional color override string. See documentation for setDefaultColor() for a list of possible color values.
   */
  void log(std::string message, std::string color);

  /**
   * logs a status message that StatusPanel can understand. This posts raw HTML as a String message.
   * @param htmlMessage the message to publish. It will not be modified in this function, just passed on.
   */
  void logHTML(std::string htmlMessage);

  ///Message queue size for publisher.
  const static unsigned int MESSAGE_QUEUE_SIZE = 100;

protected:
  ///The topic to post messages to
  std::string topic;

  ///The default color string to use when formatting.
  std::string defaultColor;

  /// The ROS publisher for the incoming messages.
  ros::Publisher publisher;

  /// The ROS node handle.
  ros::NodeHandle nh;

  /// The message to use for publishing.
  std_msgs::String msg;
};

#endif

