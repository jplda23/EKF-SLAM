execute_process(COMMAND "/home/de/catkin_ws/build/mbot_gazebo_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/de/catkin_ws/build/mbot_gazebo_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
