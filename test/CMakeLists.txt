cmake_minimum_required(VERSION 2.8.3)

# Create node_class as a testing library
add_library(node_class-lib STATIC
            ../src/main.cpp
            ../src/node_class.cpp
)
# Link node_class-lib library with catkin library
target_link_libraries(node_class-lib ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    # Find rostest package for unit testing
    find_package(rostest REQUIRED)
    # Create unit test executable
    add_rostest_gtest(node_class_utest start.test node_class_utest.cpp)
    # Link unit test executable with library
    target_link_libraries(node_class_utest ${catkin_LIBRARIES} node_class-lib)
endif()
