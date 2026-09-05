#include "doctest.h"

#include <opencv2/imgproc.hpp>

#include "DetectLines/DetectLines.h"
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

static ::cv::Mat make_lane_image( int width, int height, int left_x, int right_x )
{
  ::cv::Mat img( height, width, CV_8UC3, ::cv::Scalar( 110, 110, 110 ) );
  ::cv::line( img, { left_x, height - 1 }, { left_x, height / 2 }, ::cv::Scalar( 255, 255, 255 ), 14 );
  ::cv::line( img, { right_x, height - 1 }, { right_x, height / 2 }, ::cv::Scalar( 255, 255, 255 ), 14 );
  return img;
}

/// @brief Trapeze BEV attendu par make_lane_image (lignes de mi-hauteur au bas,
/// plein cadre) : independant de la calibration camera reelle de LaneConfig.
static void configure_synthetic_bev_trapezoid( LaneConfig& p_config )
{
  p_config.src_top_y_ratio = 0.45f;
  p_config.src_top_width_ratio = 0.18f;
  p_config.src_bottom_y_ratio = 1.0f;
  p_config.src_bottom_width_ratio = 0.5f;
  // Trapeze synthetique non representatif d'une vraie perspective camera :
  // LaneQuality::evaluate ne demoterait pas ces fits reels, seule cette
  // calibration de test approximative le fait -- ce test n'exerce pas le
  // critere de largeur de LaneQuality.
  p_config.max_width_ratio_variation = 10.0;
}

TEST_CASE( "render produit une image a la taille d'origine" )
{
  ::cv::Mat img = make_lane_image( 1280, 720, 440, 840 );
  VideoCaracteristics video( img );
  LaneConfig config;
  config.default_lane_width_px = 640.0;
  configure_synthetic_bev_trapezoid( config );
  NullImageSink sink;
  DetectLines detector( video, config, sink );

  const LaneModel model = detector.compute( img );

  ::cv::Mat out;
  detector.render( img, model, out );

  CHECK( out.size() == img.size() );
  CHECK( out.type() == img.type() );
}

TEST_CASE( "render est reproductible et ne mute pas le modele" )
{
  ::cv::Mat img = make_lane_image( 1280, 720, 440, 840 );
  VideoCaracteristics video( img );
  LaneConfig config;
  config.default_lane_width_px = 640.0;
  configure_synthetic_bev_trapezoid( config );
  NullImageSink sink;
  DetectLines detector( video, config, sink );

  const LaneModel model = detector.compute( img );
  const double offset_before = model.normalized_offset;

  ::cv::Mat first_output;
  ::cv::Mat second_output;
  detector.render( img, model, first_output );
  detector.render( img, model, second_output );

  // Le rendu a bien dessine quelque chose : sortie non vide et differente de
  // l'entree. Sans ce controle, deux rendus vides passeraient le test suivant.
  const bool first_output_is_empty = first_output.empty();
  REQUIRE( false == first_output_is_empty );

  ::cv::Mat drawn_difference;
  ::cv::absdiff( img, first_output, drawn_difference );
  const double drawn_amount = ::cv::norm( drawn_difference, ::cv::NORM_INF );
  REQUIRE( 0.0 < drawn_amount );

  // Deux rendus du meme modele sur la meme frame donnent la meme image.
  ::cv::Mat difference;
  ::cv::absdiff( first_output, second_output, difference );
  const double max_difference = ::cv::norm( difference, ::cv::NORM_INF );

  CHECK( 0.0 == max_difference );
  CHECK( offset_before == model.normalized_offset );
}

TEST_CASE( "render dessine le modele qu'on lui donne, pas celui de la frame" )
{
  // Propriete sans consommateur aujourd'hui : elle est le prerequis du lot
  // LaneTracker, qui dessinera un modele lisse sur la frame courante. Le test
  // doit donc echouer si render ignorait le modele recu.
  ::cv::Mat first_frame = make_lane_image( 1280, 720, 440, 840 );
  ::cv::Mat second_frame = make_lane_image( 1280, 720, 500, 900 );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = 640.0;
  configure_synthetic_bev_trapezoid( config );
  NullImageSink sink;
  DetectLines detector( video, config, sink );

  const LaneModel first_model = detector.compute( first_frame );
  const LaneModel second_model = detector.compute( second_frame );

  REQUIRE( first_model.lane_detected );
  REQUIRE( second_model.lane_detected );

  // Deux modeles differents rendus sur la MEME frame : les images doivent
  // differer, sinon le modele n'est pas pris en compte.
  ::cv::Mat first_render;
  ::cv::Mat second_render;
  detector.render( first_frame, first_model, first_render );
  detector.render( first_frame, second_model, second_render );

  ::cv::Mat model_difference;
  ::cv::absdiff( first_render, second_render, model_difference );
  const double model_influence = ::cv::norm( model_difference, ::cv::NORM_INF );

  CHECK( 0.0 < model_influence );

  // Et le modele d'une frame se rend sans probleme sur une autre frame.
  ::cv::Mat cross_render;
  detector.render( second_frame, first_model, cross_render );

  CHECK( cross_render.size() == second_frame.size() );
  CHECK( cross_render.type() == second_frame.type() );
}
