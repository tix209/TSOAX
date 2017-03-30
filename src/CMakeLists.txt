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


file(GLOB SRCS *.cc)
qt_wrap_cpp(${PROJECT_NAME} SRCS
  ../include/multisnake.h
  ../include/viewer.h
  ../include/main_window.h
  ../include/parameters_dialog.h
  ../include/view_options_dialog.h)

if (APPLE)
  set(ICON_FOLDER ../icon)
  set(ICON_NAME troax.icns)
  set(ICON_FILE ${ICON_FOLDER}/${ICON_NAME})
  set_source_files_properties(${ICON_FILE} PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources)

  set(COPYRIGHT "Copyright (c) 2017 Lehigh University. All rights reserved.")
  set(IDENTIFIER "edu.lehigh.troax")

  set(PROJECT_VERSION 0.1.1)
  set(MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT ${COPYRIGHT})
  set(MACOSX_BUNDLE_GUI_IDENTIFIER ${IDENTIFIER})
  set(MACOSX_BUNDLE_ICON_FILE ${ICON_NAME})
  add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${SRCS} osx_helper.mm icons.qrc
    ${ICON_FILE})
  set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST
    ${CMAKE_SOURCE_DIR}/TroaxBundleInfo.plist.in)
else (APPLE)
  add_executable(${PROJECT_NAME} ${SRCS} icons.qrc)
endif (APPLE)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${VTK_LIBRARIES})
