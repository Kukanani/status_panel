///////////////////////////////////////////////////////////////////////////////
//      Title     : Status panel
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

#ifndef STATUS_PANEL_H
#define STATUS_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QLabel;

class StatusPanel: public rviz::Panel
{
Q_OBJECT
public:
  StatusPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  void message_cb(std_msgs::String msg);

public Q_SLOTS:
  void setMessage( const QString& message );
  void setTopic();

protected:

  /// One-line text editor for entering the ROS topic to monitor for messages.
  QLineEdit* input_topic_editor;

  /// Where to display the status messages.
  QLabel* message_display;

  /// The current name of the input topic.
  QString input_topic;

  /// The ROS publisher for the incoming messages.
  ros::Subscriber subscriber;

  /// The ROS node handle.
  ros::NodeHandle nh;

};

#endif

