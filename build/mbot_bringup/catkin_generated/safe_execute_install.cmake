execute_process(COMMAND "/home/jplda23/catkin_ws/build/mbot_bringup/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jplda23/catkin_ws/build/mbot_bringup/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
