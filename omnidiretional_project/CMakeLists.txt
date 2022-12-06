project( omidirectional )
find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED )


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( omidirectional omidirectional.cpp )


target_link_libraries(omidirectional ${PCL_LIBRARIES})
target_link_libraries(omidirectional ${OpenCV_LIBS} )
