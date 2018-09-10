ADD_EXECUTABLE(run_vo run_vo.cc)
ADD_EXECUTABLE(test_frame test_frame.cc)
TARGET_LINK_LIBRARIES(test_frame myslam)
TARGET_LINK_LIBRARIES(run_vo myslam)
