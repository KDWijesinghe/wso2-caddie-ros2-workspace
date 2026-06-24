cmake_minimum_required(VERSION 3.8)
project(caddie_unitree_official)

find_package(ament_cmake REQUIRED)

install(DIRECTORY mujoco
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE.unitree_mujoco
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
