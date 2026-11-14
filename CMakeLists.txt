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

# save_data/ (SC-PGO's save_directory, the maps, the graphs) stays in the source folder, so a rebuild never touches
# it; share/map_generator/save_data links to it, so launch files find it as $(find-pkg-share map_generator)/save_data
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/save_data)
install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
  file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/save_data\")
  file(CREATE_LINK \"${CMAKE_CURRENT_SOURCE_DIR}/save_data\"
       \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/save_data\" SYMBOLIC)
")

ament_package()
