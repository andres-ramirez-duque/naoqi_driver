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

/*
 * LOCAL includes
 */
#include "animated_speech.hpp"


namespace naoqi
{
namespace subscriber
{

AnSpeechSubscriber::AnSpeechSubscriber( const std::string& name, const std::string& speech_topic, const qi::SessionPtr& session, boost::shared_ptr<tobo_planner::action_chain>& action_chain_msg):
  speech_topic_(speech_topic),
  BaseSubscriber( name, speech_topic, session ),
  action_chain_msg_( action_chain_msg ),
  p_tts_( session->service("ALAnimatedSpeech") ) //ALAnimatedSpeech - ALTextToSpeech
{}

void AnSpeechSubscriber::reset( ros::NodeHandle& nh )
{
  sub_speech_ = nh.subscribe( speech_topic_, 10, &AnSpeechSubscriber::speech_callback, this );

  is_initialized_ = true;
}

void AnSpeechSubscriber::speech_callback( const tobo_planner::action_chainConstPtr& speech_msg )
{
  action_chain_msg_->plan_step = speech_msg->plan_step;
  action_chain_msg_->action_type = speech_msg->action_type;
  action_chain_msg_->parameters = speech_msg->parameters;
  action_chain_msg_->execution_status = speech_msg->execution_status;
  if (!speech_msg->speech_cmd.empty()){
    p_tts_.async<void>("say", speech_msg->speech_cmd);
    std::cout << "Receiving topic msg: " << speech_msg->speech_cmd << std::endl;
    action_chain_msg_->execution_status.message="Speech command executed";
    action_chain_msg_->execution_status.level=0;
  }
  else{
    action_chain_msg_->execution_status.message="Speech command empty";
    action_chain_msg_->execution_status.level=1;
  }
  
}

} //publisher
} // naoqi
