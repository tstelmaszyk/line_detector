cmake_minimum_required(VERSION 3.18)
project( line_detector )
set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV REQUIRED )
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

# --- Bibliotheque : pipeline de detection ---
# Rien ici ne lit argv, n'ecrit sur stdout, ni ne possede de boucle de capture.
set(LANE_SOURCES
    src/lib/DetectLines/DetectLines.cpp
    src/lib/ImageSink/DiskImageSink.cpp
    src/lib/LanePolynomial/LanePolynomial.cpp
    src/lib/LaneGeometry/LaneGeometry.cpp
    src/lib/LaneMask/LaneMask.cpp
    src/lib/LaneQuality/LaneQuality.cpp
    src/lib/PerspectiveView/PerspectiveView.cpp
    src/lib/SlidingWindowSearch/SlidingWindowSearch.cpp
    src/lib/LaneOverlay/LaneOverlay.cpp
)

add_library( line_detector_lib STATIC ${LANE_SOURCES} )
target_include_directories( line_detector_lib PUBLIC ${CMAKE_SOURCE_DIR}/src/lib ${CMAKE_SOURCE_DIR}/src/common ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( line_detector_lib PUBLIC ${OpenCV_LIBS} )

# --- Bibliotheque applicative : capture, boucle, sorties dev/test ---
set(APP_SOURCES
    src/app/FrameObserver/AnnotatedVideoWriter.cpp
    src/app/FrameSource/CaptureFrameSource.cpp
    src/app/CliOptions/CliOptions.cpp
    src/app/FrameObserver/LaneModelLogger.cpp
    src/app/PipelineRunner/PipelineRunner.cpp
    src/app/FrameObserver/ResultImageWriter.cpp
    src/app/FrameSource/StillImageFrameSource.cpp
)

add_library( line_detector_app STATIC ${APP_SOURCES} )
target_include_directories( line_detector_app PUBLIC ${CMAKE_SOURCE_DIR}/src/app )
target_link_libraries( line_detector_app PUBLIC line_detector_lib )
target_compile_options( line_detector_app PRIVATE -Wall -Wextra )

# --- Executable ---
add_executable( line_detector src/main.cpp )
target_link_libraries( line_detector PRIVATE line_detector_app )
target_compile_options( line_detector PRIVATE -Wall -Wextra )

# --- Tests unitaires (doctest) ---
enable_testing()
file(GLOB TEST_SOURCES tests/test_*.cpp)
add_executable( line_detector_tests ${TEST_SOURCES} )
target_include_directories( line_detector_tests PRIVATE ${CMAKE_SOURCE_DIR} tests )
target_link_libraries( line_detector_tests PRIVATE line_detector_app )

# Les tests ecrivent leurs fichiers temporaires dans le dossier de build, jamais
# dans le depot. LINE_DETECTOR_TEST_TMP et TMPDIR restent prioritaires.
target_compile_definitions( line_detector_tests PRIVATE LINE_DETECTOR_TEST_TMP_DEFAULT="${CMAKE_BINARY_DIR}" )
