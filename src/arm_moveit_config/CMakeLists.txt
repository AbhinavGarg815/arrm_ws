cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs arm_moveit_config/moveit_common.py so launch files can import it.
# A loose .py next to the launch files is NOT importable: the launch system
# execs the file rather than importing it as a package member, so its directory
# never lands on sys.path.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
# Scripts must also be chmod +x in the source tree: with --symlink-install the
# installed file is a symlink and keeps the source permissions, so a
# non-executable script silently vanishes from `ros2 run`.
install(PROGRAMS
  scripts/analyse_self_collision.py
  scripts/workspace_map.py
  scripts/ik_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # joint_limits.yaml, the controller names and the named states all duplicate
  # data held elsewhere. This makes drift a build failure.
  ament_add_pytest_test(test_limits_match test/test_limits_match.py
    TIMEOUT 60
  )
endif()

ament_package()
