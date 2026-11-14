cmake_minimum_required(VERSION 3.8)
project(map_generator NONE)

find_package(ament_cmake REQUIRED)

# Python scripts, run with the python that has their dependencies (see package.xml), and the files they read next to
# them
install(PROGRAMS
  scripts/make_merged_map.py
  scripts/prune_far_graph.py
  scripts/map_point_picker.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  scripts/dynamic_removal.py
  scripts/index.html
  scripts/jet_table.npy
  DESTINATION lib/${PROJECT_NAME})

ament_package()
