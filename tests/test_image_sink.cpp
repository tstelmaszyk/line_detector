#include "doctest.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

#include "DetectLines/DetectLines.h"
#include "ImageSink/DiskImageSink.h"
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "VideoCaracteristics/VideoCaracteristics.h"
#include "test_support.h"

namespace
{

const int SINK_TEST_WIDTH = 1280;   ///< Largeur de la frame de test.
const int SINK_TEST_HEIGHT = 720;   ///< Hauteur de la frame de test.
const int SINK_LEFT_LINE_X = 440;   ///< Abscisse de la ligne gauche.
const int SINK_RIGHT_LINE_X = 840;  ///< Abscisse de la ligne droite.
const int SINK_LINE_THICKNESS = 14;    ///< Epaisseur des lignes tracees.
const int SINK_BACKGROUND_GRAY = 110;  ///< Gris du fond.
const double SINK_LANE_WIDTH_RATIO = 0.5;  ///< Largeur de voie par defaut.
const double SINK_OFFSET_TOLERANCE = 0.15;  ///< Tolerance sur l'offset normalise.

/// @brief Construit une image de voie synthetique symetrique.
::cv::Mat make_lane_frame()
  {
  const ::cv::Scalar background( SINK_BACKGROUND_GRAY, SINK_BACKGROUND_GRAY, SINK_BACKGROUND_GRAY );
  ::cv::Mat frame( SINK_TEST_HEIGHT, SINK_TEST_WIDTH, CV_8UC3, background );
  const ::cv::Scalar line_color( 255, 255, 255 );
  const ::cv::Point left_bottom( SINK_LEFT_LINE_X, SINK_TEST_HEIGHT - 1 );
  const ::cv::Point left_top( SINK_LEFT_LINE_X, SINK_TEST_HEIGHT / 2 );
  const ::cv::Point right_bottom( SINK_RIGHT_LINE_X, SINK_TEST_HEIGHT - 1 );
  const ::cv::Point right_top( SINK_RIGHT_LINE_X, SINK_TEST_HEIGHT / 2 );
  ::cv::line( frame, left_bottom, left_top, line_color, SINK_LINE_THICKNESS );
  ::cv::line( frame, right_bottom, right_top, line_color, SINK_LINE_THICKNESS );
  return frame;
  }

} // namespace

TEST_CASE( "NullImageSink : is_enabled renvoie false" )
{
  NullImageSink sink;
  const bool enabled = sink.is_enabled();

  CHECK( false == enabled );
}

TEST_CASE( "DiskImageSink : is_enabled renvoie true" )
{
  DiskImageSink sink( test_temp_dir() );
  const bool enabled = sink.is_enabled();

  CHECK( true == enabled );
}

TEST_CASE( "DetectLines::compute : meme LaneModel avec un NullImageSink" )
{
  const ::cv::Mat frame = make_lane_frame();
  VideoCaracteristics video( frame );
  LaneConfig config;
  config.default_lane_width_px = SINK_TEST_WIDTH * SINK_LANE_WIDTH_RATIO;
  // Trapeze BEV attendu par make_lane_frame (lignes de mi-hauteur au bas, plein
  // cadre) : independant de la calibration camera reelle de LaneConfig.
  config.src_top_y_ratio = 0.45f;
  config.src_top_width_ratio = 0.18f;
  config.src_bottom_y_ratio = 1.0f;
  config.src_bottom_width_ratio = 0.5f;
  // Trapeze synthetique non representatif d'une vraie perspective camera :
  // LaneQuality::evaluate ne demoterait pas ces fits reels, seule cette
  // calibration de test approximative le fait -- ce test n'exerce pas le
  // critere de largeur de LaneQuality.
  config.max_width_ratio_variation = 10.0;
  NullImageSink sink;
  const DetectLines detector( video, config, sink );

  const LaneModel model = detector.compute( frame );

  REQUIRE( model.lane_detected );
  CHECK( ::std::abs( model.normalized_offset ) < SINK_OFFSET_TOLERANCE );
}
