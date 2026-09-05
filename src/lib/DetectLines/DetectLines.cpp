/// @file
/// @brief Implémentation de DetectLines.

#include <opencv2/imgproc.hpp>

#include <vector>

#include "DetectLines/DetectLines.h"
#include "LaneGeometry/LaneGeometry.h"
#include "LanePolynomial/LanePolynomial.h"

namespace
{

const int FIT_LINE_THICKNESS = 2;                 ///< Épaisseur des polynômes tracés (debug).
const int TRAPEZE_THICKNESS = 3;                  ///< Épaisseur du trapèze source (debug).
const ::cv::Scalar COLOR_TRAPEZE( 0, 0, 255 );    ///< Rouge : trapèze source (debug).
const ::cv::Scalar COLOR_LEFT_FIT( 0, 0, 255 );   ///< Rouge : fit gauche (debug).
const ::cv::Scalar COLOR_RIGHT_FIT( 255, 0, 0 );  ///< Bleu : fit droite (debug).

} // namespace

DetectLines::DetectLines( const VideoCaracteristics& p_video,
                          const LaneConfig& p_config,
                          ImageSink& p_debug_sink )
  : m_video_properties( p_video ),
    m_config( p_config ),
    m_debug_sink( p_debug_sink ),
    m_mask( p_video, p_config, p_debug_sink ),
    m_perspective( p_video, p_config, p_debug_sink ),
    m_search( p_video, p_config, p_debug_sink ),
    m_quality( p_video, p_config, p_debug_sink ),
    m_overlay( m_perspective, p_debug_sink )
{
}

LaneModel DetectLines::compute( const ::cv::Mat& p_frame ) const
{
  const bool debug_enabled = m_debug_sink.is_enabled();

  ::cv::Mat binary;
  m_mask.compute( p_frame, binary );

  ::cv::Mat bev;
  m_perspective.to_bev( binary, bev );

  if ( debug_enabled )
    {
    m_debug_sink.save( "debug_02_bev.jpg", bev );

    // Debug calibration BEV : trapèze source sur l'image couleur + warp couleur.
    ::cv::Mat trapeze = p_frame.clone();
    const ::std::vector< ::cv::Point2f >& quad = m_perspective.source_quad();

    ::std::vector< ::cv::Point > trapeze_polygon;
    trapeze_polygon.reserve( quad.size() );

    for ( const ::cv::Point2f& corner : quad )
      {
      const int corner_x = ::cvRound( corner.x );
      const int corner_y = ::cvRound( corner.y );
      trapeze_polygon.emplace_back( corner_x, corner_y );
      }

    ::cv::polylines( trapeze, trapeze_polygon, true, COLOR_TRAPEZE, TRAPEZE_THICKNESS );
    m_debug_sink.save( "debug_02a_trapeze.jpg", trapeze );

    ::cv::Mat bev_color;
    m_perspective.to_bev( p_frame, bev_color );
    m_debug_sink.save( "debug_02b_bev_color.jpg", bev_color );
    }

  const LanePixels pixels = m_search.search( bev );

  LaneModel model;
  model.left = LanePolynomial::fit( pixels.left, m_config.window_min_pix );
  model.right = LanePolynomial::fit( pixels.right, m_config.window_min_pix );

  model = m_quality.evaluate( bev, model );

  model = LaneGeometry::compute( model, m_video_properties, m_config );

  // debug_04_fit : polynômes gauche/droite tracés sur une copie BGR de la BEV.
  const bool can_draw_fit_debug = ( debug_enabled && model.left.valid && model.right.valid );

  if ( can_draw_fit_debug )
    {
    ::cv::Mat fit_debug;
    ::cv::cvtColor( bev, fit_debug, ::cv::COLOR_GRAY2BGR );

    ::std::vector< ::cv::Point > left_points;
    ::std::vector< ::cv::Point > right_points;
    left_points.reserve( bev.rows );
    right_points.reserve( bev.rows );

    for ( int y = 0; y < bev.rows; ++y )
      {
      const int x_left = ::cvRound( model.left.eval_at( y ) );
      const int x_right = ::cvRound( model.right.eval_at( y ) );
      left_points.emplace_back( x_left, y );
      right_points.emplace_back( x_right, y );
      }

    ::cv::polylines( fit_debug, left_points, false, COLOR_LEFT_FIT, FIT_LINE_THICKNESS );
    ::cv::polylines( fit_debug, right_points, false, COLOR_RIGHT_FIT, FIT_LINE_THICKNESS );
    m_debug_sink.save( "debug_04_fit.jpg", fit_debug );
    }

  return model;
}

void DetectLines::render( const ::cv::Mat& p_frame,
                          const LaneModel& p_model,
                          ::cv::Mat& p_output ) const
{
  m_overlay.render( p_frame, p_model, p_output );
}
