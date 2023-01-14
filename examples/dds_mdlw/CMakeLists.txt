# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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

