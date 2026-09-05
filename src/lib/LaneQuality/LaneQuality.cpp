/// @file
/// @brief Implémentation de LaneQuality.

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <vector>

#include "LaneQuality/LaneQuality.h"

namespace
{

const int WIDTH_SAMPLE_COUNT = 3;               ///< Hauteurs échantillonnées : bas, milieu, haut.
const double MIN_SAMPLE_WIDTH_PX = 1.0;         ///< En dessous : les lignes sont considérées croisées.
const int REASON_BUFFER_SIZE = 160;             ///< Taille du tampon de la raison textuelle (debug).
const int FIT_LINE_THICKNESS = 2;               ///< Épaisseur des polynômes tracés (debug).
const double QUALITY_FONT_SCALE = 0.8;          ///< Échelle de la police du texte de raison (debug).
const int QUALITY_TEXT_THICKNESS = 2;           ///< Épaisseur du texte de raison (debug).
const ::cv::Point QUALITY_TEXT_ORIGIN( 20, 40 );      ///< Origine du texte de raison (debug).
const ::cv::Scalar COLOR_LEFT_FIT( 0, 0, 255 );       ///< Rouge : fit gauche d'origine (debug).
const ::cv::Scalar COLOR_RIGHT_FIT( 255, 0, 0 );      ///< Bleu : fit droit d'origine (debug).
const ::cv::Scalar COLOR_QUALITY_TEXT( 0, 0, 255 );   ///< Rouge : texte de raison (debug).

::std::array< double, WIDTH_SAMPLE_COUNT > sample_lane_widths( const LanePolynomial& p_left,
                                                                const LanePolynomial& p_right,
                                                                double p_y_max )
{
  const double y_top = 0.0;
  const double y_mid = p_y_max / 2.0;
  const double y_bottom = p_y_max;

  return {
    p_right.eval_at( y_top ) - p_left.eval_at( y_top ),
    p_right.eval_at( y_mid ) - p_left.eval_at( y_mid ),
    p_right.eval_at( y_bottom ) - p_left.eval_at( y_bottom )
  };
}

::std::string make_point_count_reason( const char* p_side, int p_point_count, int p_min_quality_points )
{
  char buffer[REASON_BUFFER_SIZE];
  ::std::snprintf( buffer,
                   sizeof( buffer ),
                   "pixels insuffisants : %s=%d < %d",
                   p_side,
                   p_point_count,
                   p_min_quality_points );
  return ::std::string( buffer );
}

::std::string make_width_reason( bool p_crosses, double p_min_width, double p_max_width, double p_max_ratio )
{
  char buffer[REASON_BUFFER_SIZE];

  if ( p_crosses )
    {
    ::std::snprintf( buffer, sizeof( buffer ), "largeur incoherente : croisement (min=%.1fpx)", p_min_width );
    }
  else
    {
    const double ratio = p_max_width / p_min_width;
    ::std::snprintf( buffer, sizeof( buffer ), "largeur incoherente : ratio %.2f > %.2f", ratio, p_max_ratio );
    }

  return ::std::string( buffer );
}

} // namespace

LaneQuality::LaneQuality( const VideoCaracteristics& p_video,
                          const LaneConfig& p_config,
                          ImageSink& p_debug_sink )
  : m_video_properties( p_video ),
    m_config( p_config ),
    m_debug_sink( p_debug_sink )
{
}

LaneModel LaneQuality::evaluate( const ::cv::Mat& p_bev, LaneModel p_model ) const
{
  const LanePolynomial original_left = p_model.left;
  const LanePolynomial original_right = p_model.right;

  ::std::string reason;

  if ( p_model.left.valid && ( p_model.left.point_count < m_config.min_quality_points ) )
    {
    p_model.left.valid = false;
    reason = make_point_count_reason( "gauche", original_left.point_count, m_config.min_quality_points );
    }

  if ( p_model.right.valid && ( p_model.right.point_count < m_config.min_quality_points ) )
    {
    p_model.right.valid = false;

    if ( reason.empty() )
      {
      reason = make_point_count_reason( "droite", original_right.point_count, m_config.min_quality_points );
      }
    }

  if ( p_model.left.valid && p_model.right.valid )
    {
    const double y_max = static_cast< double >( m_video_properties.height_pixel ) - 1.0;
    const ::std::array< double, WIDTH_SAMPLE_COUNT > widths =
      sample_lane_widths( p_model.left, p_model.right, y_max );

    double min_width = widths[0];
    double max_width = widths[0];

    for ( const double width : widths )
      {
      min_width = ::std::min( min_width, width );
      max_width = ::std::max( max_width, width );
      }

    const bool crosses = ( min_width <= MIN_SAMPLE_WIDTH_PX );
    const bool ratio_exceeded = !crosses && ( ( max_width / min_width ) > m_config.max_width_ratio_variation );

    if ( crosses || ratio_exceeded )
      {
      p_model.left.valid = false;
      p_model.right.valid = false;
      reason = make_width_reason( crosses, min_width, max_width, m_config.max_width_ratio_variation );
      }
    }

  if ( !reason.empty() && m_debug_sink.is_enabled() )
    {
    draw_debug_trace( p_bev, original_left, original_right, reason );
    }

  return p_model;
}

void LaneQuality::draw_debug_trace( const ::cv::Mat& p_bev,
                                    const LanePolynomial& p_left,
                                    const LanePolynomial& p_right,
                                    const ::std::string& p_reason ) const
{
  ::cv::Mat canvas;
  ::cv::cvtColor( p_bev, canvas, ::cv::COLOR_GRAY2BGR );

  ::std::vector< ::cv::Point > left_points;
  ::std::vector< ::cv::Point > right_points;
  left_points.reserve( p_bev.rows );
  right_points.reserve( p_bev.rows );

  for ( int y = 0; y < p_bev.rows; ++y )
    {
    if ( p_left.valid )
      {
      left_points.emplace_back( ::cvRound( p_left.eval_at( y ) ), y );
      }

    if ( p_right.valid )
      {
      right_points.emplace_back( ::cvRound( p_right.eval_at( y ) ), y );
      }
    }

  if ( !left_points.empty() )
    {
    ::cv::polylines( canvas, left_points, false, COLOR_LEFT_FIT, FIT_LINE_THICKNESS );
    }

  if ( !right_points.empty() )
    {
    ::cv::polylines( canvas, right_points, false, COLOR_RIGHT_FIT, FIT_LINE_THICKNESS );
    }

  ::cv::putText( canvas,
                p_reason,
                QUALITY_TEXT_ORIGIN,
                ::cv::FONT_HERSHEY_SIMPLEX,
                QUALITY_FONT_SCALE,
                COLOR_QUALITY_TEXT,
                QUALITY_TEXT_THICKNESS );

  m_debug_sink.save( "debug_04b_quality.jpg", canvas );
}
