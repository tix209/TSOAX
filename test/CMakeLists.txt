enable_testing()

find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED)

find_package(Qt5Test REQUIRED)

find_package(VTK REQUIRED NO_MODULE)

include(${VTK_USE_FILE})


set(common_srcs ../util.cc ../image.cc ../image_reader.cc
  ../interpolator.cc ../gradient_calculator.cc ../snake_parameters.cc
  ../multisnake.cc ../snake.cc ../junctions.cc ../snake_tip_set.cc
  ../snake_tip.cc ../munkres.cc ../snake_track.cc)

add_executable(testImageReader image_reader_test.cc ../image_reader.cc
  ../image.cc)

add_executable(testImageResampler image_resampler_test.cc ../image_resampler.cc
  ../image_reader.cc ../image.cc)

add_executable(testGradientCalculator gradient_calculator_test.cc
  ../gradient_calculator.cc ../image_reader.cc ../image.cc)

add_executable(testInterpolator interpolator_test.cc ../interpolator.cc
  ../gradient_calculator.cc ../image_reader.cc ../image.cc)

add_executable(testMultisnake multisnake_test.cc ${common_srcs} )

add_executable(testSnake snake_test.cc ../snake.cc ../image_reader.cc
  ../util.cc ../gradient_calculator.cc ../image.cc ../interpolator.cc
  ../snake_parameters.cc)

add_executable(testViewpoint viewpoint_test.cc ../viewpoint.cc)

add_executable(testImagePlane image_plane_test.cc ../image_plane.cc
  ../image_reader.cc)

add_executable(testVolumeRendering volume_rendering_test.cc
  ../volume_rendering.cc ../image_reader.cc)

add_executable(testSnakeActor snake_actor_test.cc ../snake_actor.cc
  ../snake.cc ../image_reader.cc ../util.cc ../interpolator.cc
  ../actor_color.cc ../snake_parameters.cc)

add_executable(testParametersDialog parameters_dialog_test.cc
  ../parameters_dialog.cc ../snake_parameters.cc)

add_executable(testSnakeTip snake_tip_test.cc ${common_srcs})

add_executable(testSnakeTipSet snake_tip_set_test.cc ${common_srcs})

add_executable(testJunctions junctions_test.cc ${common_srcs})

add_executable(testSnakeTrack snake_track_test.cc ${common_srcs})



target_link_libraries(testImageReader ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testImageResampler ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testGradientCalculator ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testInterpolator ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testMultisnake ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testSnake ${GTEST_BOTH_LIBRARIES} ${VTK_LIBRARIES})

target_link_libraries(testViewpoint ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testImagePlane ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testVolumeRendering ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testSnakeActor ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testParametersDialog ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES} Qt5::Widgets Qt5::Test -lpthread) # Enable libgtest
# linking

target_link_libraries(testSnakeTip ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testSnakeTipSet ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testJunctions ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

target_link_libraries(testSnakeTrack ${GTEST_BOTH_LIBRARIES}
  ${VTK_LIBRARIES})

add_custom_target(run COMMAND testImageResampler)
# add_test(TROAXTest testImageReader)

# add_test(TROAXTest testGradientCalculator)

# add_test(TROAXTest testInterpolator)

# add_test(TROAXTest testMultisnake)

# add_test(TROAXTest testSnake)

# add_test(TROAXTest testViewpoint)

# add_test(TROAXTest testImagePlane)

# add_test(TROAXTest testVolumeRendering)

# add_test(TROAXTest testSnakeActor)

# add_test(TROAXTest testParametersDialog)

# add_test(TROAXTest testSnakeTip)

# add_test(TROAXTest testSnakeTipSet)

# add_test(TROAXTest testJunctions)

# add_test(TROAXTest testSnakeTrack)
