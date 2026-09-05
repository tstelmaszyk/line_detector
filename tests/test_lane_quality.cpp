#include "doctest.h"

#include <opencv2/core.hpp>

#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneModel/LaneModel.h"
#include "LaneQuality/LaneQuality.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

namespace
{

LanePolynomial make_polynomial( double p_constant, double p_linear, double p_quadratic, int p_point_count )
{
  LanePolynomial poly;
  poly.quadratic_coefficient = p_quadratic;
  poly.linear_coefficient = p_linear;
  poly.constant_coefficient = p_constant;
  poly.point_count = p_point_count;
  poly.valid = true;
  return poly;
}

} // namespace

TEST_CASE( "paire parallele, assez de pixels -> aucune demotion" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneQuality quality( video, config, sink );

  LaneModel model;
  model.left = make_polynomial( 440.0, 0.0, 0.0, 500 );
  model.right = make_polynomial( 840.0, 0.0, 0.0, 500 );

  ::cv::Mat bev( 10, 10, CV_8UC1, ::cv::Scalar( 0 ) );
  const LaneModel result = quality.evaluate( bev, model );

  CHECK( result.left.valid );
  CHECK( result.right.valid );
}

TEST_CASE( "pixels insuffisants a gauche -> gauche demote seul" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneQuality quality( video, config, sink );

  LaneModel model;
  model.left = make_polynomial( 440.0, 0.0, 0.0, 60 );    // < min_quality_points (150 par defaut)
  model.right = make_polynomial( 840.0, 0.0, 0.0, 500 );

  ::cv::Mat bev( 10, 10, CV_8UC1, ::cv::Scalar( 0 ) );
  const LaneModel result = quality.evaluate( bev, model );

  CHECK_FALSE( result.left.valid );
  CHECK( result.right.valid );
}

TEST_CASE( "largeur incoherente sur la hauteur -> demotion des deux cotes" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneQuality quality( video, config, sink );

  // y_max = 719. left(y) = 440 (constant). right(y) = 460 + slope*y, avec
  // slope choisi pour que right(719) = 840 : largeur passe de 20px en haut
  // (y=0) a 400px en bas (y=719), ratio 20 >> max_width_ratio_variation (1.5).
  const double slope = ( 840.0 - 460.0 ) / 719.0;

  LaneModel model;
  model.left = make_polynomial( 440.0, 0.0, 0.0, 500 );
  model.right = make_polynomial( 460.0, slope, 0.0, 500 );

  ::cv::Mat bev( 10, 10, CV_8UC1, ::cv::Scalar( 0 ) );
  const LaneModel result = quality.evaluate( bev, model );

  CHECK_FALSE( result.left.valid );
  CHECK_FALSE( result.right.valid );
}

TEST_CASE( "un seul cote valide -> pas de verification de largeur" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneQuality quality( video, config, sink );

  LaneModel model;
  model.left = make_polynomial( 440.0, 0.0, 0.0, 500 );
  // model.right reste invalide par defaut (LanePolynomial() -> valid = false).

  ::cv::Mat bev( 10, 10, CV_8UC1, ::cv::Scalar( 0 ) );
  const LaneModel result = quality.evaluate( bev, model );

  CHECK( result.left.valid );
  CHECK_FALSE( result.right.valid );
}
