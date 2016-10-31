enable_testing()

# Include headers of gtest and link to the compiled library
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)
link_directories(${CMAKE_BINARY_DIR}/googletest/googlemock/gtest)


set(CMAKE_AUTOMOC ON)

# set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED)

find_package(Qt5Test REQUIRED)

find_package(VTK REQUIRED NO_MODULE)

include(${VTK_USE_FILE})


set(common_srcs ../src/util.cc ../src/image.cc ../src/image_reader.cc
  ../src/interpolator.cc ../src/gradient_calculator.cc ../src/snake_parameters.cc
  ../src/multisnake.cc ../src/snake.cc ../src/junctions.cc ../src/snake_tip_set.cc
  ../src/snake_tip.cc ../src/munkres.cc ../src/snake_track.cc)

add_executable(testImageReader image_reader_test.cc ../src/image_reader.cc
  ../src/image.cc)

add_executable(testImageResampler image_resampler_test.cc ../src/image_resampler.cc
  ../src/image_reader.cc ../src/image.cc)

add_executable(testGradientCalculator gradient_calculator_test.cc
  ../src/gradient_calculator.cc ../src/image_reader.cc ../src/image.cc)

add_executable(testInterpolator interpolator_test.cc ../src/interpolator.cc
  ../src/gradient_calculator.cc ../src/image_reader.cc ../src/image.cc)

add_executable(testMultisnake multisnake_test.cc ${common_srcs} )

add_executable(testSnake snake_test.cc ../src/snake.cc ../src/image_reader.cc
  ../src/util.cc ../src/gradient_calculator.cc ../src/image.cc ../src/interpolator.cc
  ../src/snake_parameters.cc)

add_executable(testViewpoint viewpoint_test.cc ../src/viewpoint.cc)

add_executable(testImagePlane image_plane_test.cc ../src/image_plane.cc
  ../src/image_reader.cc)

add_executable(testVolumeRendering volume_rendering_test.cc
  ../src/volume_rendering.cc ../src/image_reader.cc)

add_executable(testSnakeActor snake_actor_test.cc ../src/snake_actor.cc
  ../src/snake.cc ../src/image_reader.cc ../src/util.cc ../src/interpolator.cc
  ../src/actor_color.cc ../src/snake_parameters.cc)

add_executable(testParametersDialog parameters_dialog_test.cc
  ../src/parameters_dialog.cc ../src/snake_parameters.cc)

add_executable(testSnakeTip snake_tip_test.cc ${common_srcs})

add_executable(testSnakeTipSet snake_tip_set_test.cc ${common_srcs})

add_executable(testJunctions junctions_test.cc ${common_srcs})

add_executable(testSnakeTrack snake_track_test.cc ${common_srcs})



target_link_libraries(testImageReader gtest ${VTK_LIBRARIES})

target_link_libraries(testImageResampler gtest ${VTK_LIBRARIES})

target_link_libraries(testGradientCalculator gtest ${VTK_LIBRARIES})

target_link_libraries(testInterpolator gtest ${VTK_LIBRARIES})

target_link_libraries(testMultisnake gtest ${VTK_LIBRARIES})

target_link_libraries(testSnake gtest ${VTK_LIBRARIES})

target_link_libraries(testViewpoint gtest ${VTK_LIBRARIES})

target_link_libraries(testImagePlane gtest ${VTK_LIBRARIES})

target_link_libraries(testVolumeRendering gtest ${VTK_LIBRARIES})

target_link_libraries(testSnakeActor gtest ${VTK_LIBRARIES})

target_link_libraries(testParametersDialog gtest
  ${VTK_LIBRARIES} Qt5::Widgets Qt5::Test -lpthread) # Enable libgtest
# linking

target_link_libraries(testSnakeTip gtest ${VTK_LIBRARIES})

target_link_libraries(testSnakeTipSet gtest ${VTK_LIBRARIES})

target_link_libraries(testJunctions gtest ${VTK_LIBRARIES})

target_link_libraries(testSnakeTrack gtest ${VTK_LIBRARIES})

add_custom_target(run COMMAND testImageResampler)
