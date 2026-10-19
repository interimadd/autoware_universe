// Copyright 2026 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_IO_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_IO_HPP_

namespace autoware::motion::control::trajectory_follower_node
{
/// The ROS side of one controller: the topics it writes, the log lines it reports and the
/// diagnostics it raises.
///
/// A controller itself reads no parameter and writes no topic. Everything a controller
/// exchanges with ROS 2 is done here, in the package of the node that owns it, so that the
/// node decides what reaches the controller instead of the controller reaching out for it.
class ControllerIo
{
public:
  virtual ~ControllerIo() = default;

  /// Write out everything the controller's last cycle produced. The node calls this after
  /// a cycle that produced a command.
  virtual void publish() = 0;
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_IO_HPP_
