cmake_minimum_required(VERSION 3.5)
project(zero_arm_urdf)

# ROS2의 빌드 도구
find_package(ament_cmake REQUIRED)

# install resources (config, launch, meshes, urdf)
foreach(dir config launch meshes urdf)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()