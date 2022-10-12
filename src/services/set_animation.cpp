/*
 * Copyright 2017 Aldebaran
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

#include "set_animation.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

  SetAnimationService::SetAnimationService( const std::string& name, const std::string& topic, const qi::SessionPtr& session )
    : name_(name),
      topic_(topic),
      session_(session)
  {}

  void SetAnimationService::reset( ros::NodeHandle& nh )
  {
    service_ = nh.advertiseService(topic_, &SetAnimationService::callback, this);
  }

  bool SetAnimationService::callback( naoqi_bridge_msgs::SetStringRequest& req, naoqi_bridge_msgs::SetStringResponse& resp )
  {
    resp.success = helpers::driver::setAnimation(session_, req);
    return true;
  }


}
}
