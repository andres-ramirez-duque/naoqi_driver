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

#include "driver_helpers.hpp"

namespace naoqi
{
namespace helpers
{
namespace driver
{

/** Function that returns the type of a robot
 */
static naoqi_bridge_msgs::RobotInfo& getRobotInfoLocal( const qi::SessionPtr& session)
{
  static naoqi_bridge_msgs::RobotInfo info;
  static qi::Url robot_url;

  if (robot_url == session->url())
  {
    return info;
  }

  robot_url = session->url();

  // Get the robot type
  std::cout << "Receiving information about robot model" << std::endl;
  qi::AnyObject p_memory = session->service("ALMemory");
  std::string robot = p_memory.call<std::string>("getData", "RobotConfig/Body/Type" );
  std::string version = p_memory.call<std::string>("getData", "RobotConfig/Body/BaseVersion" );
  std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

  if (std::string(robot) == "nao")
  {
    info.type = naoqi_bridge_msgs::RobotInfo::NAO;
    std::cout << BOLDYELLOW << "Robot detected: "
              << BOLDCYAN << "NAO " << version
              << RESETCOLOR << std::endl;
  }
  if (std::string(robot) == "pepper" || std::string(robot) == "juliette" )
  {
    info.type = naoqi_bridge_msgs::RobotInfo::PEPPER;
    std::cout << BOLDYELLOW << "Robot detected: "
              << BOLDCYAN << "Pepper " << version
              << RESETCOLOR << std::endl;
  }
  if (std::string(robot) == "romeo" )
  {
    info.type = naoqi_bridge_msgs::RobotInfo::ROMEO;
    std::cout << BOLDYELLOW << "Robot detected: "
              << BOLDCYAN << "Romeo " << version
              << RESETCOLOR << std::endl;
  }

  // Get the data from RobotConfig
  qi::AnyObject p_motion = session->service("ALMotion");
  std::vector<std::vector<qi::AnyValue> > config = p_motion.call<std::vector<std::vector<qi::AnyValue> > >("getRobotConfig");

  // TODO, fill with the proper string matches from http://doc.aldebaran.com/2-1/naoqi/motion/tools-general-api.html#ALMotionProxy::getRobotConfig

  for (size_t i=0; i<config[0].size();++i)
  {
    if (config[0][i].as<std::string>() == "Model Type")
    {
      try{
        info.model = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Head Version")
    {
      try{
        info.head_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Body Version")
    {
      try{
        info.body_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Arm Version")
    {
      try{
        info.arm_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Laser")
    {
      try{
        info.has_laser = config[1][i].as<bool>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Extended Arms")
    {
      try{
        info.has_extended_arms = config[1][i].as<bool>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Legs")
    {
      try{
        info.number_of_legs = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Arms")
    {
      try{
        info.number_of_arms = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Hands")
    {
      try{
        info.number_of_hands = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

  }
  return info;
}

const robot::Robot& getRobot( const qi::SessionPtr& session )
{
  static robot::Robot robot = robot::UNIDENTIFIED;

  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::NAO )
  {
    robot = robot::NAO;
  }
  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::PEPPER )
  {
    robot = robot::PEPPER;
  }
  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::ROMEO )
  {
    robot = robot::ROMEO;
  }

  return robot;
}

const naoqi_bridge_msgs::RobotInfo& getRobotInfo( const qi::SessionPtr& session )
{
  static naoqi_bridge_msgs::RobotInfo robot_info =  getRobotInfoLocal(session);
  return robot_info;
}

/** Function that sets language for a robot
 */
bool& setLanguage( const qi::SessionPtr& session, naoqi_bridge_msgs::SetFloatRequest req)
{
  static bool success;
  std::cout << "Receiving service call of setting speech language" << std::endl;
  try{
    qi::AnyObject p_text_to_speech = session->service("ALTextToSpeech");
    //p_text_to_speech.call<void>("setLanguage", req.data);
    float fvalue = req.data;
    p_text_to_speech.call<void>("setParameter", "speed", fvalue);
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}

/** Function that gets language set to a robot
 */
std::string& getLanguage( const qi::SessionPtr& session )
{
  static std::string language;
  std::cout << "Receiving service call of getting speech language" << std::endl;
  qi::AnyObject p_text_to_speech = session->service("ALTextToSpeech");
  language = p_text_to_speech.call<std::string>("getLanguage");
  return language;
}

/**
 * Function that detects if the robot is using stereo cameras to compute depth
 */
bool isDepthStereo(const qi::SessionPtr &session) {
 std::vector<std::string> sensor_names;

 try {
   qi::AnyObject p_motion = session->service("ALMotion");
   sensor_names = p_motion.call<std::vector<std::string> >("getSensorNames");

   if (std::find(sensor_names.begin(),
                 sensor_names.end(),
                 "CameraStereo") != sensor_names.end()) {
     return true;
   }

   else {
     return false;
   }

 } catch (const std::exception &e) {
   std::cerr << e.what() << std::endl;
   return false;
 }
}

/** Function that sets Wakeup for a robot
 */
bool& setWakeup( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req)
{
  static bool success;
  std::cout << "Receiving service call of setting wakeup state on NAO" << std::endl;
  try{
    qi::AnyObject motion_service = session->service("ALMotion");
    motion_service.call<void>(req.data);
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}
/** Function that sets Leds for a robot
 */
bool& setLeds( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req)
{
  static bool success;
  std::string method;
  std::string group;
  std::cout << "Receiving service call of setting Leds on NAO" << std::endl;
  try{
        
    std::stringstream ss(req.data);
    
    std::getline(ss, method, ' ');
    std::getline(ss, group, ' ');
    
    qi::AnyObject leds_service = session->service("ALLeds");
    leds_service.call<void>(method,group);
    
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}
/** Function that sets Animation for a robot
 */
bool& setAnimation( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req)
{
  static bool success;
  std::cout << "Receiving service call of setting Animation on NAO" << std::endl;
  try{
    qi::AnyObject animation_service = session->service("ALAnimationPlayer");
    animation_service.call<void>("runTag", req.data);
    
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}
/** Function that sets Aliveness for a robot
 */
bool& setAliveness( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req)
{
  static bool success;
  std::string ability;
  std::string value;
  std::cout << "Receiving service call of setting Aliveness states on NAO" << std::endl;
  try{
    std::stringstream ss(req.data);
    
    std::getline(ss, ability, ' ');
    std::getline(ss, value, ' ');
    int ivalue = std::stoi(value);
    
    qi::AnyObject aliveness_service = session->service("ALAutonomousLife");
    aliveness_service.call<void>("setAutonomousAbilityEnabled",ability,ivalue);
    
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}

/** Function that gets running Behaviors set to a robot
 */
std::vector<std::string>& getBehavior( const qi::SessionPtr& session )
{
  static std::vector<std::string> behaviours;
  std::cout << "Receiving service call of getting running behaviors" << std::endl;
  qi::AnyObject p_behavior = session->service("ALBehaviorManager");
  behaviours = p_behavior.call<std::vector<std::string>>("getRunningBehaviors");
  return behaviours;
}
/** Function that sets ALBehaviorManager modules to a robot
 */
bool& setBehavior( const qi::SessionPtr& session )
{
  static bool success;
  std::cout << "Receiving service call of setting ALBehaviorsManager states on NAO" << std::endl;
  try{
    qi::AnyObject p_behavior = session->service("ALBehaviorManager");
    p_behavior.call<void>("stopAllBehaviors");
    std::cout << "Stoping all behaviors on NAO" << std::endl;
    qi::AnyObject p_text_to_speech = session->service("ALTextToSpeech");
    p_text_to_speech.call<void>("stopAll");
    std::cout << "Stoping all speech task" << std::endl;
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}
} // driver
} // helpers
} // naoqi
