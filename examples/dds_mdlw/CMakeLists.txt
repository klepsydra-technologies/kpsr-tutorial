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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_dds_example)
PROJECT(${PROJ_NAME})

Include (MacroConnext)
find_package (Connext REQUIRED)
SET (DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions (
  ${Connext_DEFINITIONS}
  ${DEFINITIONS}
  )

set(DDS_SOURCE_PATH "${CMAKE_SOURCE_DIR}/gen/dds")

file(REMOVE_RECURSE "${DDS_SOURCE_PATH}/gen")
file(MAKE_DIRECTORY "${DDS_SOURCE_PATH}/gen")

file(GLOB idls "${DDS_SOURCE_PATH}/idl/*.idl")
FOREACH(idl ${idls})
  Connext_IDLGEN (${idl} "${DDS_SOURCE_PATH}")
  GET_FILENAME_COMPONENT(nfile ${idl} NAME)
  Connext_GENDATAMODEL(${nfile} "${DDS_SOURCE_PATH}")
ENDFOREACH(idl)

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "./*.cpp"
)

FILE(GLOB ${PROJ_NAME}_HEADERS
    "./*.h"
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/gen/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/gen/dds/include)
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/gen/dds/gen )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${Connext_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES(${KPSR_INSTALL_PATH}/lib )

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} src/example_dds.cpp )

ADD_LIBRARY (kpsr_dds_${PROJ_MAIN_NAME}_datamodel SHARED ${Connext_DATAMODEL})
TARGET_LINK_LIBRARIES(kpsr_dds_${PROJ_MAIN_NAME}_datamodel ${Connext_LIBRARIES})

TARGET_LINK_LIBRARIES(${PROJ_NAME}
  kpsr_${PROJ_MAIN_NAME}
  kpsr_dds_${PROJ_MAIN_NAME}_datamodel
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${KLEPSYDRA_DDS_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

