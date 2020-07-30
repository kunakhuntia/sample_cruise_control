cmake_minimum_required(VERSION 3.5)
project(cruise_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)


set(dependencies
  rclcpp
  std_msgs
)

include_directories(include /opt/ros/foxy/include)

set(library_name ${PROJECT_NAME})
add_library(sensornode SHARED src/SensorNode.cpp)
add_library(algorithmnode SHARED src/AlgorithmNode.cpp)

add_executable(sensor src/Sensor.cpp)
target_link_libraries(sensor sensornode)
ament_target_dependencies(sensor ${dependencies})


add_executable(algorithm src/Algorithm.cpp)
target_link_libraries(algorithm algorithmnode) 
ament_target_dependencies(algorithm ${dependencies})

add_executable(paramserver src/ParamServer.cpp)
ament_target_dependencies(paramserver rclcpp)

add_executable(userinput src/UserInput.cpp)
ament_target_dependencies(userinput rclcpp)

install(TARGETS sensor algorithm paramserver sensornode algorithmnode userinput
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_libraries(algorithmNode)
ament_export_dependencies(${dependencies})

find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  file(GENERATE
    OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
    CONTENT "${node_plugins}")

  

  set(generated_python_files)
  macro(tests)
    set(CRUISE_CONTROL_EXE $<TARGET_FILE:sensor>)
    set(EXPECTED_OUTPUT_CC "${CMAKE_CURRENT_SOURCE_DIR}/test/pusub")
    

    set(test_names
        "Testcruisecontrol"
    )
    foreach(test_name IN LISTS test_names)
      configure_file(
        test/${test_name}.py.in
        ${test_name}${target_suffix}.py.genexp
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
      )
      add_launch_test(
        "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
        TARGET ${test_name}${target_suffix}
        ENV RMW_IMPLEMENTATION=${rmw_implementation}
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        TIMEOUT 8
      )
      list(
        APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py")
    endforeach()
  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    MAX_LINE_LENGTH 999
    ${generated_python_files})

endif()

ament_package()
