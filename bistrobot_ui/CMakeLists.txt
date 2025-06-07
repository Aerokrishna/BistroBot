cmake_minimum_required(VERSION 3.8)
project(bistrobot_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY
    backend
    backend/templates
  DESTINATION
    share/${PROJECT_NAME}/
)

set(SCRIPTS_DIR backend)
install(PROGRAMS
    ${SCRIPTS_DIR}/user_client.py

    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
