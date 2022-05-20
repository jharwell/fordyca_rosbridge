=================
fordyca_rosbridge
=================

ROS package for using `FORDYCA<https://github.com/swarm-robotics/fordyca.git>`_
with ROS.


Installation
============

#. Install catkin tools::

     pip3 install catkin_tools

#. Initialize package::

     catkin init

   If you are in a shell that has already sourced the ``setup.bash`` file for
   your ROS distro, then catkin will automatically extend that workspace for you
   when building. If you have not, you will see a warning like::

     WARNING: Your workspace is not extending any other result space, but
     it is set to use a `linked` devel space layout. This requires the
     `catkin` CMake package in your source space in order to be built.

   Which can be fixed by issuing::

     catkin config --extend /path/to/ROS/distro

   If I'm running ROS melodic installed in ``/opt``, then the command would be::

     catkin config --extend /opt/ros/melodic

#. Set catkin to install at a common location (e.g., ``$HOME/.local``)::

     catkin config --install -DCMAKE_INSTALL_PREFIX=$HOME/local

#. Build and install the package::

     catkin build
