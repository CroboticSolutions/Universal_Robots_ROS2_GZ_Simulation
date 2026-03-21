cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gz)

find_package(ament_cmake REQUIRED)

# Default to off as starting gzsim doesn't shut down correctly at the moment
option(
  UR_SIM_INTEGRATION_TESTS
  "Run ur_simulation_gz integration tests"
  OFF
)

install(DIRECTORY config doc launch urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)

  if(${UR_SIM_INTEGRATION_TESTS})
    add_launch_test(test/test_gz.py
      TIMEOUT
        180
    )
    add_launch_test(test/test_multi_ur_launch.py
      TIMEOUT
        240
    )
  endif()
  ament_add_pytest_test(description test/test_description.py)
  ament_add_pytest_test(multi_ur_parser test/test_multi_ur_parser.py)
endif()

ament_package()
