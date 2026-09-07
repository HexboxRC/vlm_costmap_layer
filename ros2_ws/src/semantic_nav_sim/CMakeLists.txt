cmake_minimum_required(VERSION 3.8)
project(semantic_nav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Share files: world/model/robot description, launch, and params.
# Directories exist from Milestone 1 (currently just .gitkeep
# placeholders) so later milestones can drop files in without touching
# this CMakeLists.
install(DIRECTORY
  launch
  worlds
  models
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# NOTE: Python trial-runner scripts will be added here as
#   install(PROGRAMS scripts/<name>.py DESTINATION lib/${PROJECT_NAME})
# once they exist (Milestone 6) — ament_cmake packages don't get an
# automatic scripts/ install, and an empty install(PROGRAMS) with no
# file arguments is a CMake error, so it's omitted until there's a
# script to list.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
