/**
 * \file crw_node.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * This file is part of ROSBRIDGE.
 *
 * ROSBRIDGE is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ROSBRIDGE is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ROSBRIDGE.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ticpp/ticpp.h>

#include "rcppsw/er/client.hpp"

#include "cosm/ros/config/sierra_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::controller::reactive::d0 {
class crw_controller;
} /* namespace fordyca::controller::reactive::d0 */
namespace fordyca::ros::support::d0 {
class d0_robot_manager;
} /* namespace fordyca::ros::support::d0 */

NS_START(rosbridge);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class crw_node
 * \ingroup
 *
 * \brief ROS node/wrapper around fcrd0::crw_controller, andol with some minimal
 * support code for sending metrics to the master node.
 */
class crw_node : public rer::client<crw_node> {
 public:
  crw_node(const cros::config::sierra_config* config,
           const ticpp::Element& root);
  ~crw_node(void);

  /* Not move/copy constructable/assignable by default */
  crw_node(const crw_node&) = delete;
  crw_node& operator=(const crw_node&) = delete;
  crw_node(crw_node&&) = delete;
  crw_node& operator=(crw_node&&) = delete;

  void run(void);

 private:
  void barrier_callback(const std_msgs::Empty::ConstPtr& msg);

  ticpp::Document load_params(const std::string& param_file);
  std::unique_ptr<fcrd0::crw_controller> controller_init(const ticpp::Element& params);
  std::unique_ptr<frsd0::d0_robot_manager> manager_init(
      const cros::config::sierra_config* config,
      const ticpp::Element& xml,
      fcrd0::crw_controller* c);

  /* clang-format off */
  const cros::config::sierra_config        mc_config;

  bool                                     m_start{false};
  std::unique_ptr<fcrd0::crw_controller>   m_controller{nullptr};
  std::unique_ptr<frsd0::d0_robot_manager> m_manager{nullptr};
  /* clang-format on */
};

NS_END(rosbridge);
