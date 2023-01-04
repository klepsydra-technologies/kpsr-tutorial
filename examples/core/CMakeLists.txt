#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************************

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_example)
PROJECT(${PROJ_NAME})

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES(${KPSR_INSTALL_PATH}/lib )

# Create Library
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp )

target_include_directories(
    ${PROJ_NAME}_1
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    
TARGET_LINK_LIBRARIES(${PROJ_NAME}_1
  kpsr_${PROJ_MAIN_NAME}
  kpsr_core
  kpsr_mem_core
  ${CMAKE_THREAD_LIBS_INIT}
)

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

target_include_directories(
    ${PROJ_NAME}_2
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  kpsr_${PROJ_MAIN_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )

target_include_directories(
    ${PROJ_NAME}_3
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  kpsr_${PROJ_MAIN_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

