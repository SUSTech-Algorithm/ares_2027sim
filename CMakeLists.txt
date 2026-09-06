cmake_minimum_required(VERSION 3.8)
project(ares_2027sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_robocon_2027_world.py
    scripts/reset_mustika.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robocon_2027_world test/test_robocon_2027_world.py)
endif()

ament_package()
