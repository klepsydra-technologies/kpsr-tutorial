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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter4)
PROJECT(${PROJ_NAME})

list(FIND KLEPSYDRA_CORE_LIBRARIES "kpsr_state_machine" KPSR_STATE_MACHINE)
if (KPSR_STATE_MACHINE EQUAL -1)
  message(FATAL_ERROR "Klepsydra SDK installed without state machine")
endif()

# Include directories with headers
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example1.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_1
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_1
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###
set(TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/data)
ADD_EXECUTABLE(${PROJ_NAME}_1a ${${PROJ_NAME}_HEADERS} src/example1a.cpp )

target_compile_definitions(${PROJ_NAME}_1a PRIVATE TEST_DATA="${TEST_DATA}")
TARGET_LINK_LIBRARIES(${PROJ_NAME}_1a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_1
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###

ADD_EXECUTABLE(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/example2.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_2
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_2
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###
ADD_EXECUTABLE(${PROJ_NAME}_3 ${${PROJ_NAME}_HEADERS} src/example3.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_3
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###

ADD_EXECUTABLE(${PROJ_NAME}_3a ${${PROJ_NAME}_HEADERS} src/example3a.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_3a
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_3a
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###
ADD_EXECUTABLE(${PROJ_NAME}_4 ${${PROJ_NAME}_HEADERS} src/example4.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_4
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_4
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###
ADD_EXECUTABLE(${PROJ_NAME}_5 ${${PROJ_NAME}_HEADERS} src/example5.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_5
  PUBLIC
  ${KLEPSYDRA_CORE_LIBRARIES}
)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_5
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
