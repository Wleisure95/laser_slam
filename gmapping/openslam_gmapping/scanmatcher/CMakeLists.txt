find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS})
add_library(scanmatcher eig3.cpp scanmatcher.cpp scanmatcherprocessor.cpp smmap.cpp)
target_link_libraries(scanmatcher sensor_range utils)

add_executable(icptest icptest.cpp)
target_link_libraries(icptest ${catkin_LIBRARIES})

install(TARGETS scanmatcher DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
