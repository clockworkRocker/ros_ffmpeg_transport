cmake_minimum_required(VERSION 3.8)
project(ffmpeg_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

OPTION(ADDR_SANITIZE OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ffmpeg_interfaces REQUIRED)
find_package(avcpp REQUIRED)

find_library(YAMLCPP_LIBRARY yaml-cpp REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${avcpp_INCLUDE_DIRS}
)

if (ADDR_SANITIZE)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -static-libasan")
endif()

# ------------------------------ CONVERTER LIB ------------------------------ #
set(PROJECT_LIBRARY_NAME ${PROJECT_NAME}_common)
file(GLOB_RECURSE ${PROJECT_LIBRARY_NAME}_SOURCES ${PROJECT_SOURCE_DIR}/src/lib/*) 
add_library(${PROJECT_LIBRARY_NAME} SHARED ${${PROJECT_LIBRARY_NAME}_SOURCES})
ament_target_dependencies(${PROJECT_LIBRARY_NAME} 
  sensor_msgs
  ffmpeg_interfaces
)
target_link_libraries(${PROJECT_LIBRARY_NAME} ${avcpp_LIBRARIES})

message("-- Using libav libraries: ${avcpp_LIBRARIES}")

# ------------------------------- ENCODER NODE ------------------------------ #
set(ENCODER_EXEC_NAME encoder_node)
file(GLOB ${ENCODER_EXEC_NAME}_SOURCES 
  src/${ENCODER_EXEC_NAME}.cpp
  src/encoding_forwarder.cpp
)
add_executable(${ENCODER_EXEC_NAME} ${${ENCODER_EXEC_NAME}_SOURCES})
ament_target_dependencies(${ENCODER_EXEC_NAME} 
  rclcpp 
  sensor_msgs
  ffmpeg_interfaces
)
target_link_libraries(${ENCODER_EXEC_NAME} 
  ${PROJECT_LIBRARY_NAME}
  ${avcpp_LIBRARIES}
  ${YAMLCPP_LIBRARY}
)

# ------------------------------- DECODER NODE ------------------------------ #
set(DECODER_EXEC_NAME decoder_node)
file(GLOB ${DECODER_EXEC_NAME}_SOURCES
  src/${DECODER_EXEC_NAME}.cpp
  src/decoding_forwarder.cpp
)
add_executable(${DECODER_EXEC_NAME} ${${DECODER_EXEC_NAME}_SOURCES})
ament_target_dependencies(${DECODER_EXEC_NAME} 
  rclcpp 
  sensor_msgs
  ffmpeg_interfaces
)
target_link_libraries(${DECODER_EXEC_NAME} 
  ${PROJECT_LIBRARY_NAME}
  ${avcpp_LIBRARIES}
  ${YAMLCPP_LIBRARY}
)

install(TARGETS
  ${ENCODER_EXEC_NAME}
  ${DECODER_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()