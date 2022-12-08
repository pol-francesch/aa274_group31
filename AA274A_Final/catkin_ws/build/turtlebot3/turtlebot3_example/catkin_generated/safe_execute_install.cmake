execute_process(COMMAND "/data/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/data/home/group38/Documents/aa274_group31/AA274A_Final/catkin_ws/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
