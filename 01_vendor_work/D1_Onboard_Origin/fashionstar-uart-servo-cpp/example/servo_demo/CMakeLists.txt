cmake_minimum_required(VERSION 3.0)
project(SERVO_QUERY_ANGLE VERSION 1.0)

SET(CMAKE_INSTALL_PREFIX "/usr/local/")

set(EXAMPLE_NAME servo_demo)

include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/bin)
add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)
target_link_libraries(${EXAMPLE_NAME}  fsuartservo cserialport)
