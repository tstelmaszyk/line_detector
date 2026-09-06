/// @file
/// @brief Implementation de LaneTracker.

#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <vector>

#include "LaneGeometry/LaneGeometry.h"
#include "LaneTracker/LaneTracker.h"
#include "SmartAssert/SmartAssert.h"

namespace
{

const int TEXT_BUFFER_SIZE = 96;                        ///< Taille du tampon texte de debug.
const int FIT_LINE_THICKNESS = 2;                       ///< Epaisseur des polynomes traces (debug).
const double TRACKER_FONT_SCALE = 0.8;                  ///< Echelle de la police du texte de debug.
const int TRACKER_TEXT_THICKNESS = 2;                   ///< Epaisseur du texte de debug.
const ::cv::Point TRACKER_TEXT_ORIGIN( 20, 40 );        ///< Origine du texte de debug.
const ::cv::Scalar COLOR_RAW_LEFT( 0, 0, 255 );         ///< Rouge : fit brut gauche (debug).
const ::cv::Scalar COLOR_RAW_RIGHT( 255, 0, 0 );        ///< Bleu : fit brut droit (debug).
const ::cv::Scalar COLOR_SMOOTHED_LEFT( 0, 255, 0 );    ///< Vert : fit lisse gauche (debug).
const ::cv::Scalar COLOR_SMOOTHED_RIGHT( 0, 255, 255 ); ///< Jaune : fit lisse droit (debug).
const ::cv::Scalar COLOR_TRACKER_TEXT( 0, 0, 255 );     ///< Rouge : texte de debug.

/// @brief Melange un coefficient brut avec sa valeur lissee precedente (EMA).
double blend( double p_previous, double p_raw, double p_alpha )
  {
  return ( p_alpha * p_raw ) + ( ( 1.0 - p_alpha ) * p_previous );
  }

/// @brief Lisse un polynome par EMA, coefficient par coefficient. Conserve
/// valid/point_count du brut (deja true/renseigne pour un cote detecte).
LanePolynomial blend_polynomial( const LanePolynomial& p_previous,
                                 const LanePolynomial& p_raw,
                                 double p_alpha )
  {
  LanePolynomial result = p_raw;
  result.quadratic_coefficient = blend( p_previous.quadratic_coefficient, p_raw.quadratic_coefficient, p_alpha );
  result.linear_coefficient = blend( p_previous.linear_coefficient, p_raw.linear_coefficient, p_alpha );
  result.constant_coefficient = blend( p_previous.constant_coefficient, p_raw.constant_coefficient, p_alpha );
  return result;
  }

} // namespace

LaneTracker::LaneTracker( const VideoCaracteristics& p_video,
                          const LaneConfig& p_config,
                          ImageSink& p_debug_sink )
  : m_video_properties( p_video ),
    m_config( p_config ),
    m_debug_sink( p_debug_sink ),
    m_smoothed_model(),
    m_has_state( false ),
    m_miss_streak( 0 )
{
  const bool alpha_in_domain = ( m_config.lane_tracker_alpha > 0.0 ) && ( m_config.lane_tracker_alpha <= 1.0 );
  SMART_ASSERT( alpha_in_domain, "LaneTracker: lane_tracker_alpha hors domaine (0 < alpha <= 1)" );
}

void LaneTracker::reset()
{
  m_smoothed_model = LaneModel();
  m_has_state = false;
  m_miss_streak = 0;
}

LaneModel LaneTracker::update( const LaneModel& p_raw_model )
{
  if ( !m_config.lane_tracker_enabled )
    {
    if ( m_debug_sink.is_enabled() )
      {
      draw_debug_trace( p_raw_model, false );
      }

    return p_raw_model;
    }

  if ( p_raw_model.lane_detected )
    {
    m_miss_streak = 0;

    LaneModel candidate_model;

    if ( !m_has_state )
      {
      candidate_model.left = p_raw_model.left;
      candidate_model.right = p_raw_model.right;
      }
    else
      {
      candidate_model.left = blend_polynomial( m_smoothed_model.left, p_raw_model.left, m_config.lane_tracker_alpha );
      candidate_model.right = blend_polynomial( m_smoothed_model.right, p_raw_model.right, m_config.lane_tracker_alpha );
      }

    candidate_model.reconstructed = p_raw_model.reconstructed;
    candidate_model = LaneGeometry::compute( candidate_model, m_video_properties, m_config );

    m_smoothed_model = candidate_model;
    m_has_state = true;

    if ( m_debug_sink.is_enabled() )
      {
      draw_debug_trace( p_raw_model, false );
      }

    return m_smoothed_model;
    }

  ++m_miss_streak;

  const bool can_coast = ( m_has_state && ( m_miss_streak <= m_config.lane_tracker_max_coast_frames ) );

  if ( can_coast )
    {
    LaneModel coasted_model = m_smoothed_model;
    coasted_model.coasted = true;

    if ( m_debug_sink.is_enabled() )
      {
      draw_debug_trace( p_raw_model, true );
      }

    return coasted_model;
    }

  m_has_state = false;

  if ( m_debug_sink.is_enabled() )
    {
    draw_debug_trace( p_raw_model, false );
    }

  return p_raw_model;
}

void LaneTracker::draw_debug_trace( const LaneModel& p_raw_model, bool p_coasted ) const
{
  ::cv::Mat canvas( m_video_properties.image_size, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );
  const int height = static_cast< int >( m_video_properties.height_pixel );

  ::std::vector< ::cv::Point > raw_left_points;
  ::std::vector< ::cv::Point > raw_right_points;
  ::std::vector< ::cv::Point > smoothed_left_points;
  ::std::vector< ::cv::Point > smoothed_right_points;

  for ( int y = 0; y < height; ++y )
    {
    if ( p_raw_model.left.valid )
      {
      raw_left_points.emplace_back( ::cvRound( p_raw_model.left.eval_at( y ) ), y );
      }

    if ( p_raw_model.right.valid )
      {
      raw_right_points.emplace_back( ::cvRound( p_raw_model.right.eval_at( y ) ), y );
      }

    if ( m_has_state )
      {
      smoothed_left_points.emplace_back( ::cvRound( m_smoothed_model.left.eval_at( y ) ), y );
      smoothed_right_points.emplace_back( ::cvRound( m_smoothed_model.right.eval_at( y ) ), y );
      }
    }

  if ( !raw_left_points.empty() )
    {
    ::cv::polylines( canvas, raw_left_points, false, COLOR_RAW_LEFT, FIT_LINE_THICKNESS );
    }

  if ( !raw_right_points.empty() )
    {
    ::cv::polylines( canvas, raw_right_points, false, COLOR_RAW_RIGHT, FIT_LINE_THICKNESS );
    }

  if ( !smoothed_left_points.empty() )
    {
    ::cv::polylines( canvas, smoothed_left_points, false, COLOR_SMOOTHED_LEFT, FIT_LINE_THICKNESS );
    }

  if ( !smoothed_right_points.empty() )
    {
    ::cv::polylines( canvas, smoothed_right_points, false, COLOR_SMOOTHED_RIGHT, FIT_LINE_THICKNESS );
    }

  char text[TEXT_BUFFER_SIZE];
  ::std::snprintf( text,
                   sizeof( text ),
                   "alpha=%.2f  miss_streak=%d  coasted=%d",
                   m_config.lane_tracker_alpha,
                   m_miss_streak,
                   p_coasted ? 1 : 0 );

  ::cv::putText( canvas,
                text,
                TRACKER_TEXT_ORIGIN,
                ::cv::FONT_HERSHEY_SIMPLEX,
                TRACKER_FONT_SCALE,
                COLOR_TRACKER_TEXT,
                TRACKER_TEXT_THICKNESS );

  m_debug_sink.save( "debug_04c_tracker.jpg", canvas );
}
