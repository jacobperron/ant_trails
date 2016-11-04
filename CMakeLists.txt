cmake_minimum_required(VERSION 2.6)

#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

include(FindPkgConfig)
pkg_search_module(STAGE REQUIRED stage)

include_directories(
  include
  ${STAGE_INCLUDE_DIRS}
)

link_directories(${STAGE_LIBRARY_DIRS})

set(PLUGIN_NAME antbot)

# create a library module for each plugin and link libstage to each
ADD_LIBRARY(${PLUGIN_NAME} MODULE src/ant_bot.cpp src/crumb.cpp)
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${STAGE_LIBRARIES})

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES(${PLUGIN_NAME} PROPERTIES PREFIX "")

# install in <prefix>/lib
#install(TARGETS ${PLUGIN_NAME} DESTINATION lib)

