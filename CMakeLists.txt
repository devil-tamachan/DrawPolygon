cmake_minimum_required(VERSION 2.8)
project (DrawPolygon)

add_definitions(-D_UNICODE -DUNICODE)

find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )

# find_package(GMP REQUIRED)
# include_directories( ${GMP_INCLUDE_DIR} )

# find_package(MPFR REQUIRED)
# include_directories( ${MPFR_INCLUDE_DIR} )

find_package( OpenCV REQUIRED )

find_package(Boost 1.64.0 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})

set(EIGEN3_DIR "" CACHE PATH "")
if(EXISTS ${EIGEN3_DIR}/Eigen/Dense)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_DIR})
elseif(EXISTS ${EIGEN3_DIR}/Dense)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_DIR}/../)
elseif(EXISTS ${EIGEN3_DIR}/include/eigen3/Eigen/Dense)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_DIR}/include/eigen3/)
endif()

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "EIGEN3_DIR not found!")
endif()

INCLUDE_DIRECTORIES( "${EIGEN3_INCLUDE_DIR}" )

add_executable(DrawPolygon
  DrawPolygon.cpp
)
target_link_libraries( DrawPolygon ${GMP_LIBRARIES} )
target_link_libraries( DrawPolygon ${MPFR_LIBRARIES} )
target_link_libraries( DrawPolygon ${OpenCV_LIBS} )
target_link_libraries( DrawPolygon ${Boost_LIBRARIES} )
