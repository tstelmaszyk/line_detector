#include "doctest.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <cstdlib>
#include <vector>

#include "DetectLines/DetectLines.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameSource/FrameSource.h"
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneModel/LaneModel.h"
#include "LaneTracker/LaneTracker.h"
#include "PipelineRunner/PipelineRunner.h"
#include "RunStats/RunStats.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

namespace
{

const int DRIFT_WIDTH = 1280;            ///< Largeur des frames.
const int DRIFT_HEIGHT = 720;            ///< Hauteur des frames.
const int DRIFT_FRAME_COUNT = 6;         ///< Frames de la sequence.
const int DRIFT_LEFT_START = 440;        ///< Abscisse initiale de la ligne gauche.
const int DRIFT_RIGHT_START = 840;       ///< Abscisse initiale de la ligne droite.
const int DRIFT_STEP_PX = 25;            ///< Deplacement des lignes entre deux frames.
const int DRIFT_LINE_THICKNESS = 14;     ///< Epaisseur des lignes tracees.
const int DRIFT_BACKGROUND_GRAY = 110;   ///< Gris du fond.
const double DRIFT_LANE_WIDTH_RATIO = 0.5;   ///< Largeur de voie par defaut.
const double DRIFT_MONOTONY_TOLERANCE = 0.02;  ///< Tolerance de non-monotonie locale.
const double DRIFT_MINIMUM_TOTAL = 0.10;       ///< Variation minimale attendue sur la sequence.

/// @brief Construit une frame ou les deux lignes sont decalees vers la droite.
///
/// Les marquages qui glissent vers la droite = vehicule de plus en plus a gauche
/// de la voie, donc normalized_offset qui decroit (negatif = decale a gauche).
::cv::Mat make_drifting_frame( int p_frame_index )
  {
  const ::cv::Scalar background( DRIFT_BACKGROUND_GRAY, DRIFT_BACKGROUND_GRAY, DRIFT_BACKGROUND_GRAY );
  ::cv::Mat frame( DRIFT_HEIGHT, DRIFT_WIDTH, CV_8UC3, background );
  const ::cv::Scalar line_color( 255, 255, 255 );
  const int shift = p_frame_index * DRIFT_STEP_PX;
  const int left_x = DRIFT_LEFT_START + shift;
  const int right_x = DRIFT_RIGHT_START + shift;
  const ::cv::Point left_bottom( left_x, DRIFT_HEIGHT - 1 );
  const ::cv::Point left_top( left_x, DRIFT_HEIGHT / 2 );
  const ::cv::Point right_bottom( right_x, DRIFT_HEIGHT - 1 );
  const ::cv::Point right_top( right_x, DRIFT_HEIGHT / 2 );
  ::cv::line( frame, left_bottom, left_top, line_color, DRIFT_LINE_THICKNESS );
  ::cv::line( frame, right_bottom, right_top, line_color, DRIFT_LINE_THICKNESS );
  return frame;
  }

/// @brief Source servant la sequence de derive, a partir de la 2e frame.
class DriftingFrameSource : public FrameSource
  {
  public:
    /// @brief Construit la source.
    DriftingFrameSource()
      : m_next_index( 1 )
      {
      }

    /// @brief Sert la frame suivante de la sequence.
    bool read( ::cv::Mat& p_frame ) override
      {
      if ( DRIFT_FRAME_COUNT <= m_next_index )
        {
        return false;
        }

      p_frame = make_drifting_frame( m_next_index );
      ++m_next_index;
      return true;
      }

  private:
    int m_next_index;  ///< Index de la prochaine frame a servir.
  };

/// @brief Observateur memorisant l'offset normalise de chaque frame.
class OffsetRecorder : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    OffsetRecorder()
      : m_offsets()
      {
      }

    /// @brief Memorise l'offset de la frame.
    void on_frame( int p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   double p_compute_ms,
                   double p_render_ms ) override
      {
      (void) p_frame_index;
      (void) p_annotated_frame;
      (void) p_compute_ms;
      (void) p_render_ms;
      REQUIRE( p_model.lane_detected );
      m_offsets.push_back( p_model.normalized_offset );
      }

    /// @brief Offsets memorises, dans l'ordre des frames.
    const ::std::vector< double >& offsets() const
      {
      return m_offsets;
      }

  private:
    ::std::vector< double > m_offsets;  ///< Offsets memorises.
  };

} // namespace

TEST_CASE( "mode video : une voie qui derive produit un offset monotone" )
{
  const ::cv::Mat first_frame = make_drifting_frame( 0 );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = DRIFT_WIDTH * DRIFT_LANE_WIDTH_RATIO;
  // Trapeze BEV attendu par make_drifting_frame (lignes de mi-hauteur au bas,
  // plein cadre) : independant de la calibration camera reelle de LaneConfig.
  config.src_top_y_ratio = 0.45f;
  config.src_top_width_ratio = 0.18f;
  config.src_bottom_y_ratio = 1.0f;
  config.src_bottom_width_ratio = 0.5f;
  // Trapeze synthetique non representatif d'une vraie perspective camera :
  // LaneQuality::evaluate ne demoterait pas ces fits reels, seule cette
  // calibration de test approximative le fait -- ce test n'exerce pas le
  // critere de largeur de LaneQuality.
  config.max_width_ratio_variation = 10.0;
  // Tracker desactive : ce test verifie la derive du signal BRUT a travers
  // SlidingWindowSearch/LaneGeometry, pas le lissage (couvert par test_lane_tracker.cpp).
  config.lane_tracker_enabled = false;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  DriftingFrameSource source;
  OffsetRecorder recorder;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &recorder );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  REQUIRE( EXIT_SUCCESS == status );
  REQUIRE( DRIFT_FRAME_COUNT == stats.frame_count );
  REQUIRE( DRIFT_FRAME_COUNT == static_cast< int >( recorder.offsets().size() ) );

  const ::std::vector< double >& offsets = recorder.offsets();

  // L'offset decroit d'une frame a l'autre (tolerance sur le bruit de detection).
  for ( ::std::size_t offset_index = 1; offset_index < offsets.size(); ++offset_index )
    {
    const double previous_offset = offsets[offset_index - 1];
    const double current_offset = offsets[offset_index];
    CHECK( current_offset <= ( previous_offset + DRIFT_MONOTONY_TOLERANCE ) );
    }

  // La derive totale est nettement superieure au bruit.
  const double first_offset = offsets.front();
  const double last_offset = offsets.back();
  CHECK( last_offset < ( first_offset - DRIFT_MINIMUM_TOTAL ) );
}
