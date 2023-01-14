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


SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_tests)
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/gen/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )

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
ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

TARGET_LINK_LIBRARIES(${PROJ_NAME}
  kpsr_${PROJ_MAIN_NAME}
  ${KLEPSYDRA_CORE_LIBRARIES}
  gtest_main)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME})
