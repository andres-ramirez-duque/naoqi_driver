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


#ifndef ANSPEECH_SUBSCRIBER_HPP
#define ANSPEECH_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tobo_planner/action_chain.h>

namespace naoqi
{
namespace subscriber
{

class AnSpeechSubscriber: public BaseSubscriber<AnSpeechSubscriber>
{
public:
  AnSpeechSubscriber( const std::string& name, const std::string& speech_topic, const qi::SessionPtr& session, boost::shared_ptr<tobo_planner::action_chain>& action_chain_msg);
  ~AnSpeechSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void speech_callback( const tobo_planner::action_chainConstPtr& speech_msg );

private:

  std::string speech_topic_;
  
  qi::AnyObject p_tts_;
  ros::Subscriber sub_speech_;
  boost::shared_ptr<tobo_planner::action_chain> action_chain_msg_;


}; // class Speech

} // subscriber
}// naoqi
#endif
