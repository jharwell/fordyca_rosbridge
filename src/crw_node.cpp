/**
 * \file crw_node.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>
#include <ticpp/ticpp.h>
#include <fstream>

#include "fordyca/controller/reactive/d0/crw_controller.hpp"

#include "cosm/ros/topic.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

NS_END(fordyca);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
void ros_init(int argc, char** argv) {
  ros::init(argc, argv, "crw");
}

std::unique_ptr<fcrd0::crw_controller> controller_init(void) {
  ros::NodeHandle nh("~");
  std::string param_file;
  auto ros_ns = ::ros::this_node::getNamespace();

  auto id = rtypes::type_uuid(std::atoi(ros_ns.c_str() +
                                        1 +
                                        cpal::kRobotNamePrefix.size() +
                                        1));
  auto pal_ns = cros::to_ns(id);
  ROS_ASSERT_MSG(ros_ns == pal_ns,
                 "ROS robot namespace != configure PAL robot prefix: %s != %s",
                 ros_ns.c_str(),
                 pal_ns.c_str());
  nh.getParam("/sierra/experiment/param_file", param_file);
  ROS_INFO("Loading CRW params from %s", param_file.c_str());

  std::ifstream in;
  in.open(param_file, std::ios::in);
  ticpp::Document doc;
  in >> doc;

  auto crw = std::make_unique<fcrd0::crw_controller>();
  crw->entity_id(id);
  auto* root = doc.FirstChildElement();
  auto* crw_params = root->FirstChildElement("crw_controller");
  crw->init(*crw_params);
  return crw;
} /* controller_init() */

int main(int argc, char** argv) {
  /* initialize ROS */
  ros_init(argc, argv);

  /* initialize CRW controller */
  auto crw = controller_init();

  /* loop forever */
  size_t i = 0;
  while (::ros::ok()) {
    /* !!!
     * This is only here for initial bringup; actually belongs in the loop
     * functions
     * !!!
     */
    crw->sensing_update(rtypes::timestep(i++),
                               rtypes::discretize_ratio(0.2));
    crw->control_step();
    ::ros::spinOnce();
  } /* while() */

  return 0;
} /* main() */
