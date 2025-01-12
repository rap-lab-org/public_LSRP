cmake_minimum_required(VERSION 2.8.3)
project(LSRP)

find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB_RECURSE ALL_HDRS "include/*.hpp")
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")


include_directories(DIRECTORY
  include
)

add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
        include/mapfaa_util.hpp
        include/mapfaa_lsrp.hpp
        source/mapfaa_lsrp.cpp
        include/mapfaa_util.hpp
        source/mapfaa_util.cpp
)

# ------------------------------------------------------------------------------
add_executable(lsrp
        source/main.cpp
)

target_link_libraries(lsrp
        ${PROJECT_NAME}
        ${CMAKE_THREAD_LIBS_INIT}
)

set(test_cpp_dir "test/")
file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")
foreach(test_cpp_file ${test_cpp_files})
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)
  add_executable(${test_cpp_name} ${test_cpp_file})
  target_link_libraries(${test_cpp_name}
          ${PROJECT_NAME}
          ${CMAKE_THREAD_LIBS_INIT}
  )
endforeach(test_cpp_file ${test_cpp_files})
# ------------------------------------------------------------------------------
#set(test_cpp_dir "test/")
#set(test_cpp_files "")
#file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")
#foreach(test_cpp_file ${test_cpp_files})
#  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)
#  ADD_EXECUTABLE(${test_cpp_name} ${test_cpp_dir}/${test_cpp_name}.cpp
#          test/test_movingai.cpp
#  )
#
#  TARGET_LINK_LIBRARIES(${test_cpp_name}
#    ${PROJECT_NAME}
#    ${CMAKE_THREAD_LIBS_INIT}
#  )
#endforeach(test_cpp_file ${test_cpp_files})
