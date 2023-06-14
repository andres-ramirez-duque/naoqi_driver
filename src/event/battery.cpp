/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

#include "battery.hpp"

namespace naoqi
{

BatteryEventRegister::BatteryEventRegister()
{
}


BatteryEventRegister::BatteryEventRegister( const std::string& key, const qi::SessionPtr& session, boost::shared_ptr<tobo_planner::action_chain>& msg )
  : key_(key),
    p_memory_( session->service("ALMemory") ),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    msg_(msg),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<tobo_planner::action_chain> >( key_ );
  converter_ = boost::make_shared<converter::BatteryEventConverter>( key_, 0, session);
  
  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<tobo_planner::action_chain>::publish, publisher_, _1) );
  
  signal_ = p_memory_.call<qi::AnyObject>("subscriber", key_);
}

BatteryEventRegister::~BatteryEventRegister()
{
  stopProcess();
}

void BatteryEventRegister::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

void BatteryEventRegister::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

void BatteryEventRegister::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    registerCallback();
    isStarted_ = true;
  }
}

void BatteryEventRegister::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    unregisterCallback();
    isStarted_ = false;
  }
}

void BatteryEventRegister::writeDump(const ros::Time& time)
{
  
}

void BatteryEventRegister::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

void BatteryEventRegister::isRecording(bool state)
{
  //boost::mutex::scoped_lock rec_lock(mutex_);
  //isRecording_ = state;
}

void BatteryEventRegister::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

void BatteryEventRegister::isDumping(bool state)
{
  //boost::mutex::scoped_lock dump_lock(mutex_);
  //isDumping_ = state;
}

void BatteryEventRegister::registerCallback()
{
  signalID_ = signal_.connect("signal", (boost::function<void (qi::AnyValue&)>(boost::bind(&BatteryEventRegister::onEvent, this, _1))));
  std::cout << key_ << " : Start" << std::endl;
}

void BatteryEventRegister::unregisterCallback()
{
  signal_.disconnect(signalID_);
  std::cout << key_ << " : Stop" << std::endl;
}

void BatteryEventRegister::onEvent(qi::AnyValue &value)//std::string &key, qi::AnyValue &value, qi::AnyValue &message - std::string &eventName, int &percentage, std::string &subscriberIdentifier
{
  tobo_planner::action_chain msg = tobo_planner::action_chain();
  msg.caller_id = msg_->caller_id;
  msg.speech_cmd = msg_->speech_cmd;
  msg.plan_step = msg_->plan_step;
  msg.action_type = msg_->action_type;
  msg.parameters = msg_->parameters;
  msg.execution_status.level = value.asInt32();
  msg.execution_status.name = "NAO battery level";
  msg.execution_status.message = "NAO battery percentage level changed";
  
  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);
  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    if (actions.size() >0)
    {
      converter_->callAll( actions, msg );
    }
  }
}

}//namespace
