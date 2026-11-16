cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Scripts must also be chmod +x in the source tree: with --symlink-install the
# installed file is a symlink and keeps the source permissions.
install(PROGRAMS
  scripts/capture_zero.py
  scripts/make_collision_meshes.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Parses the xacro and asserts the things that have actually bitten us:
  # every joint limited, one hardware component, the base frame upright.
  ament_add_pytest_test(test_description test/test_description.py
    TIMEOUT 120
  )
  # Config files are data; data gets validated. See the test's docstring for
  # the silent RViz failure that made this necessary.
  ament_add_pytest_test(test_configs_parse test/test_configs_parse.py
    TIMEOUT 60
  )
endif()

ament_package()
