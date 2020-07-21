add_executable(pr2 pr2.cpp)
target_link_libraries(pr2 ${catkin_LIBRARIES})

add_executable(pr2_bimodal pr2_bimodal.cpp)
target_link_libraries(pr2_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pr2_dual_arm_test pr2_dual_arm_test.cpp)
target_link_libraries(pr2_dual_arm_test ${catkin_LIBRARIES})

add_executable(pr2_handover pr2_handover.cpp)
target_link_libraries(pr2_handover ${catkin_LIBRARIES})

add_executable(pr2_merger pr2_merger.cpp)
target_link_libraries(pr2_merger ${catkin_LIBRARIES})

add_executable(pr2_bimanual_v2 pr2_bimanual_v2.cpp)
target_link_libraries(pr2_bimanual_v2 ${PROJECT_NAME}_stages ${catkin_LIBRARIES})

add_executable(pr2_place pr2_place.cpp)
target_link_libraries(pr2_place ${catkin_LIBRARIES})

add_executable(pr2_parallel pr2_parallel.cpp)
target_link_libraries(pr2_parallel ${catkin_LIBRARIES})


add_executable(pr2_cartesian pr2_cartesian.cpp)
target_link_libraries(pr2_cartesian ${catkin_LIBRARIES})

add_executable(pr2_detect_stall pr2_detect_stall.cpp)
target_link_libraries(pr2_detect_stall ${catkin_LIBRARIES})

add_executable(pr2_create_task pr2_create_task.cpp)
target_link_libraries(pr2_create_task ${catkin_LIBRARIES})

add_executable(pr2_torso pr2_torso.cpp)
target_link_libraries(pr2_torso ${catkin_LIBRARIES})

add_executable(pr2_base pr2_base.cpp)
target_link_libraries(pr2_base ${catkin_LIBRARIES})

add_executable(pr2_gripper pr2_gripper.cpp)
target_link_libraries(pr2_gripper ${catkin_LIBRARIES})

