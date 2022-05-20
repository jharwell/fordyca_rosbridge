/**
 * \file crw_node.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca_rosbridge/crw_node.hpp"

#include <fstream>

#include "cosm/ros/topic.hpp"
#include "cosm/ros/config/server/sierra_parser.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/ros/support/d0/d0_robot_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(rosbridge);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
crw_node::crw_node(const cros::config::sierra_config* config,
                   const ticpp::Element& root)
    : ER_CLIENT_INIT("rosbridge.crw_node"),
      mc_config(*config),
      m_controller(controller_init(root)),
      m_manager(manager_init(config, root, m_controller.get())) {}

crw_node::~crw_node(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<fcrd0::crw_controller> crw_node::controller_init(
    const ticpp::Element& params) {
  ::ros::NodeHandle nh("~");

  auto ros_ns = ::ros::this_node::getNamespace();

  auto id = rtypes::type_uuid(std::atoi(ros_ns.c_str() +
                                        1 + /* move past '/' */
                                        cpal::kRobotNamePrefix.size()));
  ER_INFO("Computed ROS robot id=%d, ROS namespace=%s",
          id.v(),
          ros_ns.c_str());
  auto pal_ns = cros::to_ns(id);
  ROS_ASSERT_MSG(ros_ns == pal_ns,
                 "ROS robot namespace != configure PAL robot prefix: %s != %s",
                 ros_ns.c_str(),
                 pal_ns.c_str());


  auto crw = std::make_unique<fcrd0::crw_controller>();
  crw->entity_id(id);

  auto* controller_params = params.FirstChildElement("controllers");
  auto* crw_params = controller_params->FirstChildElement("crw_controller");
  crw->init(*crw_params);
  return crw;
} /* controller_init() */

std::unique_ptr<frsd0::d0_robot_manager> crw_node::manager_init(
    const cros::config::sierra_config* config,
    const ticpp::Element& xml,
    fcrd0::crw_controller* c) {
  auto robot_ns = ::ros::this_node::getNamespace();
  auto lf = std::make_unique<frsd0::d0_robot_manager>(robot_ns, config, c);

  auto* root = xml.FirstChildElement("loop_functions");
  lf->init(*root);

  return lf;
} /* manager_init() */

void crw_node::barrier_callback(const std_msgs::Empty::ConstPtr& msg) {
  ER_INFO("Received experiment start signal");
  m_start = true;
} /* barrier_callback() */

void crw_node::run(void) {
  /* set controller rate */
  ::ros::Rate rate(mc_config.experiment.ticks_per_sec.v());

  ER_INFO("Experiment start barrier: %d", mc_config.experiment.barrier_start);

  if (mc_config.experiment.barrier_start) {
    ::ros::NodeHandle nh;
    /* leading '/' -> we require this message to be sent from the master node */
    auto sub = nh.subscribe("/sierra/experiment/start",
                            1000,
                            &crw_node::barrier_callback,
                            this);
    while (::ros::ok() && !m_start) {
      ER_DEBUG("Waiting for signal to start experiment");
      ::ros::spinOnce();
      ::ros::Duration(1.0).sleep();
    }
  }
  ER_INFO("Beginning main loop");
  /* loop until done */
  while (::ros::ok() && !m_manager->experiment_finished()) {
    /* update current tick and other such tasks */
    m_manager->pre_step();

    /* run controller */
    m_controller->control_step();

    /* collect metrics from controller */
    m_manager->post_step();

    /*
     * IMPORTANT! rate.sleep() only sleeps the thread and does NOT cause ROS to
     * handle publisher/subscriber updates, so we need to do both.
     */
    rate.sleep();
    ::ros::spinOnce();
  } /* while() */

  ER_INFO("Finished main loop");

  m_manager->destroy();
} /* run() */

NS_END(rosbridge);


/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
void ros_init(int argc, char** argv) {
  ::ros::init(argc, argv, "crw");
}

int main(int argc, char** argv) {
  /* initialize ROS */
  ros_init(argc, argv);

  /* get XML parameters for controller and loop functions */
  ::ros::NodeHandle nh("~");
  cros::config::server::sierra_parser sierra;
  sierra.parse();
  auto* config = sierra.config_get<cros::config::sierra_config>();

  ROS_INFO("Loading parameters from %s", config->experiment.param_file.c_str());
  std::ifstream in;
  in.open(config->experiment.param_file, std::ios::in);
  ticpp::Document doc;
  in >> doc;
  auto* root = doc.FirstChildElement();

  /* Create node, spinning to let everything come up */
  rosbridge::crw_node node(config, *root);
  ::ros::spinOnce();

  /* GO GO POWER RANGERS! */
  node.run();

  /* all done! */
  ROS_INFO("Initiating crw_node shutdown");
  ::ros::shutdown();

  return 0;
} /* main() */
