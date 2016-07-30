cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

project(TROAX)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  if(MSVC) # MSVC compiler (Win32 only)
    # Display more warnings
    set(CMAKE_CXX_FLAGS "/W3")
  elseif(UNIX OR CMAKE_COMPILER_IS_GNUCXX)
    # Clang OR Gcc (Linux, Mac OS or Win32 with MingW)
    # Displays all warnings
    set(CMAKE_CXX_FLAGS "-O0 -pedantic -Wall")
  endif()
endif()

if(UNIX OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# add_subdirectory(test)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

find_package(Qt5Widgets REQUIRED)
find_package(VTK REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

set(srcs troax.cc main_window.cc viewer.cc image_reader.cc viewpoint.cc
  image_plane.cc slice_planes.cc volume_rendering.cc multisnake.cc snake.cc
  image.cc gradient_calculator.cc interpolator.cc util.cc snake_actor.cc
  snake_parameters.cc parameters_dialog.cc junctions.cc snake_tip_set.cc
  snake_tip.cc junction_actor.cc actor_color.cc munkres.cc
  view_options_dialog.cc snake_track.cc image_resampler.cc)

if (APPLE)
  set(ICON_FOLDER icon)
  set(ICON_NAME troax.icns)
  set(ICON_FILE ${ICON_FOLDER}/${ICON_NAME})
  set_source_files_properties(${ICON_FILE} PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources)

  set(COPYRIGHT "Copyright (c) 2016 Lehigh University. All rights reserved.")
  set(IDENTIFIER "edu.lehigh.troax")

  set(MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_LONG_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT ${COPYRIGHT})
  set(MACOSX_BUNDLE_GUI_IDENTIFIER ${IDENTIFIER})
  set(MACOSX_BUNDLE_ICON_FILE ${ICON_NAME})
  add_executable(troax MACOSX_BUNDLE ${srcs} icons.qrc ${ICON_FILE} osx_helper.mm)
else (APPLE)
  add_executable(troax ${srcs} icons.qrc)
endif (APPLE)

target_link_libraries(troax Qt5::Widgets ${VTK_LIBRARIES})
