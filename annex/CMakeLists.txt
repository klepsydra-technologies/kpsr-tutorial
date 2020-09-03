find_package(Klepsydra)

cmake_minimum_required(VERSION 2.8.11)
SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_chapter1)
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${KLEPSYDRA_INCLUDE_DIRS} )

ADD_EXECUTABLE(${PROJ_NAME}_7 ${${PROJ_NAME}_HEADERS} src/example7.cpp )

TARGET_LINK_LIBRARIES(${PROJ_NAME}_7
  ${KLEPSYDRA_CORE_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  atomic
  )
