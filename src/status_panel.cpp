///////////////////////////////////////////////////////////////////////////////
//      Title     : Status logger panel
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
////////////////////////////////////////////////////////////////////////////////

#include "status_panel/status_panel.h"

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

namespace status_panel {

StatusPanel::StatusPanel( QWidget* parent ) :
  rviz::Panel( parent ),
  input_topic("/status")
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Status Topic:" ));
  input_topic_editor = new QLineEdit;
  topic_layout->addWidget( input_topic_editor );

  message_display = new QLabel("");
  message_display->setTextFormat(Qt::RichText);
  message_display->setAlignment(Qt::AlignCenter);

  // Lay out the topic field next to the control widrivzget.
  QGridLayout* layout = new QGridLayout();
  layout->setColumnStretch(1,100);
  layout->addWidget( message_display, 0,0,1,4);
  layout->addLayout( topic_layout, 0,4,1,1 );
  setLayout( layout );

  input_topic_editor->resize(150, input_topic_editor->height());

  // Next we make signal/slot connections.
  connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));

  input_topic_editor->setText( input_topic );
  setTopic();
}

void StatusPanel::setTopic()
{
  if(subscriber) {
    subscriber.shutdown();
  }
  input_topic = input_topic_editor->text();
  subscriber = nh.subscribe(std::string(input_topic.toStdString()), 100, &StatusPanel::message_cb, this);
  Q_EMIT configChanged();
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void StatusPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "topic", input_topic );
}

// Load all configuration data for this panel from the given Config object.
void StatusPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "topic", &topic ))
  {
    input_topic_editor->setText( topic );
    setTopic();
  }
}

void StatusPanel::setMessage( const QString& msg) {
  message_display->setText(QString("<span style='font-weight: bold; font-size: 14pt;'>") + msg + "</span>");
}

void StatusPanel::message_cb(std_msgs::String msg)
{
  setMessage(QString(msg.data.c_str()));
}


} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(status_panel::StatusPanel,rviz::Panel )
