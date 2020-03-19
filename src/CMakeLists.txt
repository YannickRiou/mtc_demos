add_executable(pr2 pr2.cpp)
target_link_libraries(pr2 ${catkin_LIBRARIES})

add_executable(pr2_bimodal pr2_bimodal.cpp)
target_link_libraries(pr2_bimodal ${catkin_LIBRARIES} gtest)
