# Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

  find_package (Qt5 COMPONENTS Widgets Gui)

  include(YarpUseQt5)
  include(YarpOSXUtilities)

  include_directories(${YARP_INCLUDE_DIRS})

  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

  set(textimage_SRCS main.cpp display.cpp)

  set(textimage_HDRS display.h)

  set(textimage_QRC_FILES res.qrc)

  set(textimage_UI_FILES display.ui)

  qt5_add_resources(textimage_QRC_GEN_SRCS ${textimage_QRC_FILES})

  qt5_wrap_ui(textimage_UI_GEN_SRCS ${textimage_UI_FILES})

  source_group("Source Files" FILES ${textimage_SRCS})
  source_group("Header Files" FILES ${textimage_HDRS})
  source_group("Resources Files" FILES ${textimage_QRC_FILES})
  source_group("UI Files" FILES ${textimage_UI_FILES})
  source_group("Generated Files" FILES ${textimage_QRC_GEN_SRCS}
                                       ${textimage_UI_GEN_SRCS})

  add_executable(textimage ${textimage_SRCS}
                                      ${textimage_HDRS}
                                      ${textimage_QRC_GEN_SRCS}
                                      ${textimage_UI_GEN_SRCS})

  target_link_libraries(textimage ${YARP_LIBRARIES})

  qtyarp_use_modules(textimage Widgets  Gui)

  qtyarp_deprecate_with_cmake_version(2.8.11)

  install(TARGETS textimage COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

  yarp_osx_duplicate_and_add_bundle(TARGET_ORIG textimage
                                    TARGET_DEST textimage.app
                                    INSTALL_COMPONENT utilities
                                    INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}")

