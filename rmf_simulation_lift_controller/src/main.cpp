/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <rclcpp/rclcpp.hpp>

#include <rmf_simulation_lift_controller/SimulatedLiftController.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // A single-threaded executor serializes all callbacks, so the allocator
  // state does not need any locking.
  rclcpp::spin(
    std::make_shared<rmf_simulation_lift_controller::SimulatedLiftController>());

  rclcpp::shutdown();
  return 0;
}
