/// @file
/// @brief Implémentation de LaneModelLogger.

#include <iomanip>
#include <ostream>
#include <string>

#include "FrameObserver/LaneModelLogger.h"
#include "LaneModel/LaneModel.h"

namespace
{

const char FIELD_SEPARATOR = ';';  ///< Séparateur de champs.

const ::std::string CSV_HEADER =
  "frame_index;lane_detected;normalized_offset;lateral_offset_px;"
  "curvature_radius_px;reconstructed;coasted;compute_ms;render_ms";  ///< En-tête du log.

const ::std::streamsize CSV_FIELD_PRECISION = 6;  ///< Decimales des champs flottants (format fixe).

} // namespace

LaneModelLogger::LaneModelLogger( ::std::ostream& p_output_stream )
  : m_output_stream( p_output_stream ),
    m_header_written( false )
{
  // Format fixe a precision constante : evite le bascule en notation
  // scientifique sur un grand curvature_radius_px et stabilise les colonnes.
  m_output_stream << ::std::fixed << ::std::setprecision( CSV_FIELD_PRECISION );
}

void LaneModelLogger::on_frame( FrameIndex p_frame_index,
                                const LaneModel& p_model,
                                const ::cv::Mat& p_annotated_frame,
                                DurationMs p_compute_ms,
                                DurationMs p_render_ms )
{
  (void) p_annotated_frame;

  if ( !m_header_written )
    {
    m_output_stream << CSV_HEADER << "\n";
    m_header_written = true;
    }

  const int lane_detected_flag = p_model.lane_detected ? 1 : 0;
  const int reconstructed_flag = p_model.reconstructed ? 1 : 0;
  const int coasted_flag = p_model.coasted ? 1 : 0;

  m_output_stream << p_frame_index << FIELD_SEPARATOR
                  << lane_detected_flag << FIELD_SEPARATOR
                  << p_model.normalized_offset << FIELD_SEPARATOR
                  << p_model.lateral_offset_px << FIELD_SEPARATOR
                  << p_model.curvature_radius_px << FIELD_SEPARATOR
                  << reconstructed_flag << FIELD_SEPARATOR
                  << coasted_flag << FIELD_SEPARATOR
                  << p_compute_ms << FIELD_SEPARATOR
                  << p_render_ms << "\n";
}
