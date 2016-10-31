#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

find_package(Qt5Widgets REQUIRED)
find_package(VTK REQUIRED NO_MODULE)
include(${VTK_USE_FILE})


file(GLOB SRCS *.cc)
qt_wrap_cpp(troax SRCS
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

  set(COPYRIGHT "Copyright (c) 2016 Lehigh University. All rights reserved.")
  set(IDENTIFIER "edu.lehigh.troax")

  set(MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_LONG_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT ${COPYRIGHT})
  set(MACOSX_BUNDLE_GUI_IDENTIFIER ${IDENTIFIER})
  set(MACOSX_BUNDLE_ICON_FILE ${ICON_NAME})
  add_executable(troax MACOSX_BUNDLE ${SRCS} osx_helper.mm icons.qrc
    ${ICON_FILE})
else (APPLE)
  add_executable(troax ${SRCS} ../icon/icons.qrc)
endif (APPLE)

target_link_libraries(troax Qt5::Widgets ${VTK_LIBRARIES})
