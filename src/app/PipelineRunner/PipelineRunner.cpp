/// @file
/// @brief Implémentation de PipelineRunner.

#include <chrono>
#include <cstdlib>
#include <vector>

#include "LaneModel/LaneModel.h"
#include "PipelineRunner/PipelineRunner.h"

namespace
{

const double MICROSECONDS_PER_MILLISECOND = 1000.0;  ///< Conversion µs -> ms.

/// @brief Duree ecoulee depuis un instant de depart, en millisecondes.
/// @param p_start_time Instant de depart.
/// @return Duree ecoulee (millisecondes).
DurationMs elapsed_ms_since( const ::std::chrono::steady_clock::time_point& p_start_time )
  {
  const ::std::chrono::steady_clock::time_point end_time = ::std::chrono::steady_clock::now();
  const ::std::chrono::microseconds elapsed_us =
    ::std::chrono::duration_cast< ::std::chrono::microseconds >( end_time - p_start_time );
  const DurationMs elapsed_ms = static_cast< DurationMs >( elapsed_us.count() ) / MICROSECONDS_PER_MILLISECOND;
  return elapsed_ms;
  }

} // namespace

PipelineRunner::PipelineRunner( FrameSource& p_frame_source,
                                const DetectLines& p_detector,
                                LaneTracker& p_tracker,
                                const ::std::vector< FrameObserver* >& p_observers,
                                const ::std::atomic< bool >& p_stop_requested )
  : m_frame_source( p_frame_source ),
    m_detector( p_detector ),
    m_tracker( p_tracker ),
    m_observers( p_observers ),
    m_stop_requested( p_stop_requested ),
    m_render_needed( false )
{
  // Le besoin de rendu est fige a la construction : la liste d'observateurs
  // ne change pas pendant un run.
  for ( const FrameObserver* observer : m_observers )
    {
    if ( nullptr != observer )
      {
      const bool observer_needs_frame = observer->needs_annotated_frame();

      if ( observer_needs_frame )
        {
        m_render_needed = true;
        }
      }
    }
}

bool PipelineRunner::process_frame( const ::cv::Mat& p_frame, FrameIndex p_frame_index, RunStats& p_stats )
{
  const ::std::chrono::steady_clock::time_point compute_start = ::std::chrono::steady_clock::now();

  const LaneModel raw_model = m_detector.compute( p_frame );
  const LaneModel model = m_tracker.update( raw_model );

  const DurationMs compute_ms = elapsed_ms_since( compute_start );

  ::cv::Mat annotated_frame;
  DurationMs render_ms = 0.0;

  if ( m_render_needed )
    {
    const ::std::chrono::steady_clock::time_point render_start = ::std::chrono::steady_clock::now();

    m_detector.render( p_frame, model, annotated_frame );

    render_ms = elapsed_ms_since( render_start );
    }

  bool has_fatal_error = false;

  for ( FrameObserver* observer : m_observers )
    {
    if ( nullptr != observer )
      {
      observer->on_frame( p_frame_index, model, annotated_frame, compute_ms, render_ms );
      const bool observer_has_fatal_error = observer->has_fatal_error();

      if ( observer_has_fatal_error )
        {
        has_fatal_error = true;
        }
      }
    }

  ++p_stats.frame_count;
  p_stats.compute_ms = p_stats.compute_ms + compute_ms;
  p_stats.render_ms = p_stats.render_ms + render_ms;

  if ( model.lane_detected )
    {
    ++p_stats.detected_count;
    }

  if ( model.reconstructed )
    {
    ++p_stats.reconstructed_count;
    }

  return has_fatal_error;
}

int PipelineRunner::run( const ::cv::Mat& p_first_frame, RunStats& p_stats )
{
  ::cv::Mat current_frame = p_first_frame;
  ::cv::Mat next_frame;
  FrameIndex frame_index = 0;

  while ( true )
    {
    const bool frame_is_empty = current_frame.empty();

    if ( frame_is_empty )
      {
      break;
      }

    const bool observer_has_fatal_error = process_frame( current_frame, frame_index, p_stats );
    ++frame_index;

    if ( observer_has_fatal_error )
      {
      break;
      }

    // Un arret demande est constate apres la frame en cours, pour ne pas la perdre.
    const bool stop_after_frame = m_stop_requested.load();

    if ( stop_after_frame )
      {
      break;
      }

    const bool read_ok = m_frame_source.read( next_frame );

    if ( !read_ok )
      {
      break;
      }

    current_frame = next_frame;
    }

  const bool any_frame_processed = ( 0 < p_stats.frame_count );

  if ( !any_frame_processed )
    {
    return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
