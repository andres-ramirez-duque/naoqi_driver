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

#ifndef ENDSPEECH_EVENT_REGISTER_HPP
#define ENDSPEECH_EVENT_REGISTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/end_speech.hpp"
// Publisher
#include "../src/publishers/basic.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/

class EndSpeechEventRegister
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  EndSpeechEventRegister();
  EndSpeechEventRegister(const std::string& key, const qi::SessionPtr& session, boost::shared_ptr<tobo_planner::action_chain>& msg );
  ~EndSpeechEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  boost::shared_ptr<converter::EndSpeechEventConverter> converter_;
  boost::shared_ptr<publisher::BasicPublisher<tobo_planner::action_chain>> publisher_;
  //boost::shared_ptr<Recorder> recorder_;

  qi::AnyObject p_memory_;
  qi::AnyObject signal_;
  qi::SignalLink signalID_;
  std::string key_;
  boost::shared_ptr<tobo_planner::action_chain> msg_;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

}; // class globalrecorder
} //naoqi

#endif
