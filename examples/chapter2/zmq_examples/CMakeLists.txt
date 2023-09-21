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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter2)
project(${PROJ_NAME})

list(FIND KLEPSYDRA_ZMQ_LIBRARIES "kpsr_zmq_core" KPSR_ZMQ)
if(KPSR_ZMQ EQUAL -1)
    message(
        FATAL_ERROR
            "Klepsydra Core installed without ZMQ. \
Please reinstall by enabling ZMQ support (passing -DKPSR_WITH_ZMQ=true to cmake options)"
    )
endif()

# Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")
file(GLOB ${PROJ_NAME}_HEADERS "include/*.h")

# Create executables
# ---------------------------------------------------#
# Add executables
# ---------------------------------------------------#
add_executable(${PROJ_NAME}_1 ${${PROJ_NAME}_HEADERS} src/example_zmq.cpp)
add_executable(${PROJ_NAME}_2 ${${PROJ_NAME}_HEADERS} src/environment_zmq.cpp)

# Link libraries
# ---------------------------------------------------#
target_link_libraries(${PROJ_NAME}_1 PUBLIC ${KLEPSYDRA_ZMQ_LIBRARIES})
target_link_libraries(${PROJ_NAME}_2 PUBLIC ${KLEPSYDRA_ZMQ_LIBRARIES})

# Include directories
# ---------------------------------------------------#
target_include_directories(${PROJ_NAME}_1
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
