.. index:: pair: page; Adding new ROS message definitions
.. _doxid-tutorial_add_ros_msg_definition:

Adding new ROS message definitions
==================================

This guide shows how to extend the ROS message definitions available in NRP-core. Afterwards, the new definitions become available to use in TransceiverFunctions and ROS Engines.

The process is very simple and consists of these steps:

#. Add your new message definitions into a separate ``.msg`` file. Refer to `creating a ROS msg guide <http://wiki.ros.org/action/show/msg?action=show&redirect=ROS%2FMessage_Description_Language>`__ to check available possibilities.

#. Place the new ``.msg`` file in the folder ``nrp_ros/nrp_ros_msgs/nrp_ros_msgs/msg``. In this way your message definitions will be automatically compiled during the NRP-core build process. Also the Python bindings required (see :ref:`ROS msg datapacks <doxid-datapacks_1datapacks_rosmsg>` section) to use these new msg classes in tranceiver functions will be generated and compiled.

#. Add the new ``.msg`` to ``add_message_files()`` FILES in ``nrp_ros/nrp_ros_msgs/nrp_ros_msgs/CMakeLists.txt`` so it is compiled:
   
   .. ref-code-block:: cpp
   
   	add_message_files(
   	  FILES
   	  Test.msg
   	)

#. If needed, extend DEPENDENCIES in generates_messages() function with the necessary packages your messages depend on:
   
   .. ref-code-block:: cpp
   
   	generate_messages(DEPENDENCIES
   	  std_msgs
   	  geometry_msgs
   	)

#. Again only if needed, add these dependencies to ``package.xml`` :
   
   .. ref-code-block:: cpp
   
   	<build_depend>std_msgs</build_depend>
   	<run_depend>std_msgs</run_depend>

