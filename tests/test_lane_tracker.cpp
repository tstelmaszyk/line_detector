#include "doctest.h"

#include <opencv2/core.hpp>

#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneGeometry/LaneGeometry.h"
#include "LaneModel/LaneModel.h"
#include "LaneTracker/LaneTracker.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

namespace
{

/// @brief Polynome constant (voie verticale), comme dans test_lane_geometry.cpp.
LanePolynomial straight( double p_x )
{
  LanePolynomial poly;
  poly.quadratic_coefficient = 0.0;
  poly.linear_coefficient = 0.0;
  poly.constant_coefficient = p_x;
  poly.valid = true;
  poly.point_count = 500;
  return poly;
}

/// @brief Construit un LaneModel brut "detecte", comme le ferait DetectLines::compute
/// (fit + LaneGeometry::compute), pour nourrir LaneTracker::update avec une entree realiste.
LaneModel make_detected_model( double p_left_x,
                               double p_right_x,
                               const VideoCaracteristics& p_video,
                               const LaneConfig& p_config )
{
  LaneModel model;
  model.left = straight( p_left_x );
  model.right = straight( p_right_x );
  return LaneGeometry::compute( model, p_video, p_config );
}

/// @brief Construit un LaneModel brut "non detecte" (aucun cote valide).
LaneModel make_missed_model( const VideoCaracteristics& p_video, const LaneConfig& p_config )
{
  LaneModel model;
  return LaneGeometry::compute( model, p_video, p_config );
}

} // namespace

TEST_CASE( "LaneTracker desactive : pass-through strict, aucun etat accumule" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_enabled = false;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  const LaneModel first_raw = make_detected_model( 440.0, 840.0, video, config );
  const LaneModel first_result = tracker.update( first_raw );

  CHECK( first_result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( first_result.right.constant_coefficient == doctest::Approx( 840.0 ) );
  CHECK_FALSE( first_result.coasted );

  const LaneModel second_raw = make_detected_model( 460.0, 860.0, video, config );
  const LaneModel second_result = tracker.update( second_raw );

  // Pas d'etat accumule : le second resultat est exactement le second brut,
  // pas un melange avec le premier appel.
  CHECK( second_result.left.constant_coefficient == doctest::Approx( 460.0 ) );
  CHECK( second_result.right.constant_coefficient == doctest::Approx( 860.0 ) );
}

TEST_CASE( "LaneTracker : premiere detection -> seed identite" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  const LaneModel raw = make_detected_model( 440.0, 840.0, video, config );
  const LaneModel result = tracker.update( raw );

  REQUIRE( result.lane_detected );
  CHECK( result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 840.0 ) );
  CHECK_FALSE( result.coasted );
}

TEST_CASE( "LaneTracker : deuxieme detection -> EMA vers la nouvelle mesure" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_alpha = 0.3;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );  // seed
  const LaneModel result = tracker.update( make_detected_model( 540.0, 940.0, video, config ) );

  // smoothed = alpha*raw + (1-alpha)*previous = 0.3*540 + 0.7*440 = 470 (idem +30 a droite).
  CHECK( result.left.constant_coefficient == doctest::Approx( 470.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 870.0 ) );
  CHECK_FALSE( result.coasted );
}

TEST_CASE( "LaneTracker : perte courte -> coasting sur le dernier modele lisse" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_max_coast_frames = 3;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );
  const LaneModel result = tracker.update( make_missed_model( video, config ) );

  REQUIRE( result.lane_detected );
  CHECK( result.coasted );
  CHECK( result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 840.0 ) );
}

TEST_CASE( "LaneTracker : perte prolongee -> reset puis reseed identite" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_max_coast_frames = 2;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );       // seed
  tracker.update( make_missed_model( video, config ) );                       // miss 1/2 : coasting
  tracker.update( make_missed_model( video, config ) );                       // miss 2/2 : coasting
  const LaneModel after_reset = tracker.update( make_missed_model( video, config ) );  // miss 3 : reset

  CHECK_FALSE( after_reset.lane_detected );
  CHECK_FALSE( after_reset.coasted );

  // Le prochain fit valide reseed a l'identite, pas de melange avec l'etat perime.
  const LaneModel reseeded = tracker.update( make_detected_model( 900.0, 1300.0, video, config ) );
  CHECK( reseeded.left.constant_coefficient == doctest::Approx( 900.0 ) );
  CHECK( reseeded.right.constant_coefficient == doctest::Approx( 1300.0 ) );
}

TEST_CASE( "LaneTracker : reset() manuel efface l'etat" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );
  tracker.reset();
  const LaneModel result = tracker.update( make_detected_model( 900.0, 1300.0, video, config ) );

  CHECK( result.left.constant_coefficient == doctest::Approx( 900.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 1300.0 ) );
}
