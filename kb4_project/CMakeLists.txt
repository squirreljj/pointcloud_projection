project( kb4_project )
find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED )


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( kb4_project kb4_project.cpp )


target_link_libraries(kb4_project ${PCL_LIBRARIES})
target_link_libraries(kb4_project ${OpenCV_LIBS} )
