cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    message_filters
    nav_msgs
    std_msgs
    tf
    tf2 
    tf2_ros
    geometry_msgs
    dynamic_reconfigure
)

add_message_files(
  FILES
    MotorSpeed.msg
    FourSpeeds.msg
    Speeds.msg
    OdometryAndMethod.msg
)

add_service_files(
  FILES
    Pose.srv
    Null.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/dynamic_reconfig.cfg)

catkin_package(
  CATKIN_DEPENDS
    message_filters
    std_msgs
    nav_msgs
    message_runtime
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(synchro_motor src/synchro_motor.cpp)
target_link_libraries(synchro_motor ${catkin_LIBRARIES})
add_dependencies(synchro_motor robotics_hw1_generate_messages_cpp)

add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(gearbox_estimator src/gearbox_estimator.cpp)
target_link_libraries(gearbox_estimator ${catkin_LIBRARIES})

add_executable(baseline_estimator src/baseline_estimator.cpp)
target_link_libraries(baseline_estimator ${catkin_LIBRARIES})

add_executable(synchro_baseline_estimator src/synchro_baseline_estimator.cpp)
target_link_libraries(synchro_baseline_estimator ${catkin_LIBRARIES})

add_executable(differential_drive_velocities src/differential_drive_velocities.cpp)
target_link_libraries(differential_drive_velocities ${catkin_LIBRARIES})

add_executable(compute_odom src/compute_odom.cpp)
add_dependencies(compute_odom ${PROJECT_NAME}_gencfg)
add_dependencies(compute_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(compute_odom ${catkin_LIBRARIES})




