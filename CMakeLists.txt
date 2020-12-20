cmake_minimum_required(VERSION 3.5)
project(diffbot2_simulation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/diffbot2.test.py)
endif()

# Install share files
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()