cmake_minimum_required(VERSION 3.8)
project(kpbot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY
    launch
    maps
    params
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/nav2_api.py
    ${SCRIPTS_DIR}/joy_control.py
    ${SCRIPTS_DIR}/odometry.py
    ${SCRIPTS_DIR}/wheel_control.py
    ${SCRIPTS_DIR}/joy_speed_control.py
    ${SCRIPTS_DIR}/bldc_control.py





    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
