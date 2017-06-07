project(pid-controller)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(uWebSockets REQUIRED)
find_package(NLohmannJSON REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(IPOPT REQUIRED)
find_package(CppAD REQUIRED)

if(APPLE)
  find_package(OpenSSL REQUIRED)
  find_package(LibUV REQUIRED)
endif()

include_directories(
  ${UWEBSOCKETS_INCLUDE_DIR}
  ${NLOHMANNJSON_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CPPAD_INCLUDE_DIR}
  ${OPENSSL_INCLUDE_DIR})

add_definitions(-std=c++0x)

add_executable(mpc
  src/main.cc
  src/utils.cc
  src/poly.cc
  src/mpc.cc)

target_link_libraries(
  mpc
  ssl z
  ${UWEBSOCKETS_LIBRARIES}
  ${LIBUV_LIBRARY}
  ${IPOPT_LIBRARIES})
