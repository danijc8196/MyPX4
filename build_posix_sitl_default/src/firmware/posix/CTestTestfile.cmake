# CMake generated Testfile for 
# Source directory: /home/dani/ROS_WS/src/MyPX4/src/firmware/posix
# Build directory: /home/dani/ROS_WS/src/MyPX4/build_posix_sitl_default/src/firmware/posix
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(rcS_tests "/home/dani/ROS_WS/src/MyPX4/Tools/sitl_run.sh" "/home/dani/ROS_WS/src/MyPX4/build_posix_sitl_default/src/firmware/posix/px4" "posix-configs/SITL/init/test" "none" "none" "none" "/home/dani/ROS_WS/src/MyPX4" "/home/dani/ROS_WS/src/MyPX4/build_posix_sitl_default")
set_tests_properties(rcS_tests PROPERTIES  PASS_REGULAR_EXPRESSION "All tests passed" WORKING_DIRECTORY "/home/dani/ROS_WS/src/MyPX4/build_posix_sitl_default/tmp")
