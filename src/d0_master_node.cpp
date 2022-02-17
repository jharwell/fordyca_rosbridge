/**
 * \file d0_master_node.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "d0_master_node.hpp"

#include <ros/ros.h>
#include <ticpp/ticpp.h>
#include <fstream>

#include "cosm/ros/topic.hpp"
#include "cosm/ros/config/server/sierra_parser.hpp"

#include "fordyca/ros/support/d0/d0_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(rosbridge);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d0_master_node::d0_master_node(const cros::config::sierra_config* config,
                   const ticpp::Element& root)
    : ER_CLIENT_INIT("rosbridge.d0_master_node"),
      mc_config(*config),
      m_manager(manager_init(config, root)) {}

d0_master_node::~d0_master_node(void) = default;


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<frsd0::d0_swarm_manager> d0_master_node::manager_init(
    const cros::config::sierra_config* config,
    const ticpp::Element& xml) {
  auto lf = std::make_unique<frsd0::d0_swarm_manager>(config);

  auto* root = xml.FirstChildElement("loop_functions");
  lf->init(*root);

  return lf;
} /* manager_init() */

void d0_master_node::run(void) {
  /* set loop function rate */
  ::ros::Rate rate(mc_config.experiment.ticks_per_sec.v());

  /*
   * When we get here, all robots are up (we have subscribed to all their
   * metrics), so we are OK to start the experiment.
   */
  ER_INFO("Experiment start barrier: %d", mc_config.experiment.barrier_start);
  if (mc_config.experiment.barrier_start) {
    ::ros::NodeHandle nh;
    auto pub = nh.advertise<std_msgs::Empty>("/sierra/experiment/start",
                                             10,
                                             true);
    pub.publish(std_msgs::Empty());
    ::ros::spinOnce();
    ::ros::Duration(1.0).sleep();
  }

  /* loop until done */
  ER_INFO("Entering main loop");
  while (!m_manager->experiment_finished()) {
    if (!::ros::ok()) {
      ER_ERR("::ros::ok() failed--exiting early");
      break;
    }

    /* update current tick and other such tasks */
    m_manager->pre_step();

    /* collect metrics all controllers */
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
  ::ros::init(argc, argv, "d0_master");
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

  /* initialize loop functions */
  rosbridge::d0_master_node node(config, *root);
  ::ros::spinOnce();

  node.run();

  /* all done! */
  ROS_INFO("Initiating d0_master_node shutdown");
  ::ros::shutdown();

  return 0;
} /* main() */
