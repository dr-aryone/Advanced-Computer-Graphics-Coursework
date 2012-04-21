cmake_minimum_required (VERSION 2.6)
project (hw3)


# all the .cpp files that make up this project
add_executable(render
  main.cpp
  matrix.cpp
  camera.cpp
  glCanvas.cpp
  mesh.cpp
  edge.cpp
  radiosity.cpp
  face.cpp
  raytree.cpp
  raytracer.cpp
  sphere.cpp
  cylinder_ring.cpp
  material.cpp
  image.cpp
  photon_mapping.cpp
  kdtree.cpp
  MersenneTwister.h
  argparser.h
  boundingbox.h
  camera.h
  cylinder_ring.h
  edge.h
  face.h
  glCanvas.h
  hash.h
  hit.h
  image.h
  kdtree.h
  material.h
  matrix.h
  mesh.h
  photon.h
  photon_mapping.h
  primitive.h
  radiosity.h
  ray.h
  raytracer.h
  raytree.h
  sphere.h
  utils.h
  vectors.h
  vertex.h
)


# platform specific compiler flags to output all compiler warnings
if (UNIX)
  if (${CMAKE_SYSTEM_NAME} STREQUAL "FreeBSD")
    set_target_properties (render PROPERTIES COMPILE_FLAGS "-g -Wall -pedantic -DFreeBSD")
  else()
    set_target_properties (render PROPERTIES COMPILE_FLAGS "-g -Wall -pedantic -std=c++0x")
  endif()
endif()

if (APPLE)
set_target_properties (render PROPERTIES COMPILE_FLAGS "-g -Wall -pedantic") 
# -m32")
endif()

if (WIN32)
set_target_properties (render PROPERTIES COMPILE_FLAGS "/W4")
endif()



# a work-around function to handle a list of libraries that include a
#  NOTFOUND library
function (add_lib_list target liblist)
  foreach (lib ${liblist})
    if (lib)
      target_link_libraries(${target} "${lib}")
    else()
      message(STATUS "WARNING: missing library: ${lib}")
    endif()
  endforeach()
endfunction()



# search for the GL & GLUT & GLEW libraries

find_package(GLUT)
if (NOT GLUT_FOUND)
  message(FATAL_ERROR "Cannot find GLUT library")
endif()
message(STATUS "Found GLUT at \"${GLUT_LIBRARIES}\"")

find_package(OpenGL)
if (NOT OPENGL_FOUND)
   message(FATAL_ERROR "Cannot find OpenGL library")
endif()
message(STATUS "Found OpenGL at \"${OPENGL_LIBRARIES}\"")

add_lib_list(render "${OPENGL_LIBRARIES}")
add_lib_list(render "${GLUT_LIBRARIES}")

if (WIN32)
  find_library(GLEW_LIBRARIES glew32 HINT "lib")
  if (NOT GLEW_LIBRARIES)
    message(FATAL_ERROR "Cannot find GLEW library")
  endif()
  message(STATUS "Found GLEW at \"${GLEW_LIBRARIES}\"")
  add_lib_list(render "${GLEW_LIBRARIES}")
endif()

#include_directories(".")
