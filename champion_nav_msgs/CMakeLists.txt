cmake_minimum_required(VERSION 2.8.3)
project(champion_nav_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
	std_msgs
        geometry_msgs
        nav_msgs
        )


#message
add_message_files(
    FILES
    ChampionNavLaserScan.msg
)


#message dependence
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
#  DEPENDS system_lib

)
