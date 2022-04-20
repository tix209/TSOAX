set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# A preferred way to setup Qt5 dependencies. See more at
# https://blog.kitware.com/cmake-finding-qt5-the-right-way/. Before configure,
# Qt5_DIR should be set by 'export Qt5_DIR="/usr/local/opt/qt5"' in your
# .bashrc. Set CMAKE_PREFIX_PATH is discouraged.
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

## Enable these options for Windows build
if (WIN32)
  set(Boost_USE_STATIC_LIBS        ON)
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME    OFF)
endif()

find_package(Boost COMPONENTS
  program_options filesystem system REQUIRED)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif(Boost_FOUND)

#file(GLOB SRCS *.cc)
set(SRCS actor_color.cc
        analysis_options_dialog.cc
        gradient_calculator.cc
        grid.cc
        image.cc
        image_plane.cc
        image_reader.cc
        image_resampler.cc
        interpolator.cc
        junction_actor.cc
        junctions.cc
        lapjv.cc
        main_window.cc
        multisnake.cc
        munkres.cc
        parameters_dialog.cc
        slice_planes.cc
        snake.cc
        snake_actor.cc
        snake_parameters.cc
        snake_tip.cc
        snake_tip_set.cc
        snake_track.cc
        util.cc
        view_options_dialog.cc
        viewer.cc
        viewpoint.cc
        volume_rendering.cc)

qt_wrap_cpp(${PROJECT_NAME} SRCS
  ../include/multisnake.h
  ../include/viewer.h
  ../include/main_window.h
  ../include/parameters_dialog.h
  ../include/view_options_dialog.h
  ../include/analysis_options_dialog.h)

if (APPLE)
  set(ICON_FOLDER ../icon)
  set(ICON_NAME troax.icns)
  set(ICON_FILE ${ICON_FOLDER}/${ICON_NAME})
  set_source_files_properties(${ICON_FILE} PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources)

  set(COPYRIGHT "Copyright (c) 2018 Lehigh University. All rights reserved.")
  set(IDENTIFIER "edu.lehigh.tsoax")

  set(PROJECT_VERSION 0.1.3)
  set(MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT ${COPYRIGHT})
  set(MACOSX_BUNDLE_GUI_IDENTIFIER ${IDENTIFIER})
  set(MACOSX_BUNDLE_ICON_FILE ${ICON_NAME})
  add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${SRCS} icons.qrc
    ${ICON_FILE})
  set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST
    ${CMAKE_SOURCE_DIR}/TroaxBundleInfo.plist.in)
else (APPLE)
  add_executable(${PROJECT_NAME} ${SRCS} troax.cc icons.qrc)
endif (APPLE)

add_executable(batch_tsoax ${SRCS} batch_tsoax.cc )

target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${VTK_LIBRARIES})
target_link_libraries(batch_tsoax Qt5::Widgets ${VTK_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
