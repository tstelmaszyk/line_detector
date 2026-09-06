#include "doctest.h"

#include "LaneConfig/LaneConfig.h"

TEST_CASE( "LaneConfig fournit des valeurs par defaut saines" )
{
  LaneConfig config;

  CHECK( config.window_count > 0 );
  CHECK( config.window_margin > 0 );
  CHECK( config.window_min_pix > 0 );
  CHECK( config.white_threshold > 0 );
  CHECK( config.src_top_width_ratio > 0.0f );
  CHECK( config.src_top_y_ratio > 0.0f );
  CHECK( config.src_top_y_ratio < 1.0f );
  CHECK( config.src_bottom_width_ratio > 0.0f );
  CHECK( config.src_bottom_width_ratio <= 0.5f );
  CHECK( config.src_bottom_y_ratio > 0.0f );
  CHECK( config.src_bottom_y_ratio <= 1.0f );
  CHECK( config.default_lane_width_px == doctest::Approx( 0.0 ) );

  CHECK( config.min_quality_points > 0 );
  CHECK( config.min_quality_points > config.window_min_pix );
  CHECK( config.max_width_ratio_variation > 1.0 );

  CHECK( config.lane_tracker_enabled );
  CHECK( config.lane_tracker_alpha > 0.0 );
  CHECK( config.lane_tracker_alpha <= 1.0 );
  CHECK( config.lane_tracker_max_coast_frames > 0 );
}
