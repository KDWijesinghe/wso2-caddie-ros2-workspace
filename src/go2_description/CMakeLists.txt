cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config dae meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE.unitree_ros README.unitree.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
