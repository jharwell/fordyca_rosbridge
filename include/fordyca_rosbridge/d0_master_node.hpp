/**
 * \file crw_node.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * This file is part of ROSBRIDGE.
 *
 * ROSBRIDGE is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
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
class d0_swarm_manager;
} /* namespace fordyca::ros::support::d0 */

NS_START(rosbridge);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class d0_master_node
 * \ingroup
 *
 * \brief
 */
class d0_master_node : public rer::client<d0_master_node> {
 public:
  d0_master_node(const cros::config::sierra_config* config,
                 const ticpp::Element& xml);
  ~d0_master_node(void);

  /* Not move/copy constructable/assignable by default */
  d0_master_node(const d0_master_node&) = delete;
  d0_master_node& operator=(const d0_master_node&) = delete;
  d0_master_node(d0_master_node&&) = delete;
  d0_master_node& operator=(d0_master_node&&) = delete;

  void run(void);

 private:
  std::unique_ptr<frsd0::d0_swarm_manager> manager_init(
      const cros::config::sierra_config* config,
      const ticpp::Element& xml);

  /* clang-format off */
  const cros::config::sierra_config        mc_config;

  std::unique_ptr<frsd0::d0_swarm_manager> m_manager{nullptr};
  /* clang-format on */
};

NS_END(rosbridge);
