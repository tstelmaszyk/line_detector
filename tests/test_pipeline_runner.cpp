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

const int RUNNER_TEST_WIDTH = 640;        ///< Largeur des frames de test.
const int RUNNER_TEST_HEIGHT = 360;       ///< Hauteur des frames de test.
const int RUNNER_TEST_FRAME_COUNT = 4;    ///< Frames servies par la source factice.
const int RUNNER_LEFT_LINE_X = 220;       ///< Abscisse de la ligne gauche.
const int RUNNER_RIGHT_LINE_X = 420;      ///< Abscisse de la ligne droite.
const int RUNNER_LINE_THICKNESS = 10;     ///< Epaisseur des lignes tracees.
const int RUNNER_BACKGROUND_GRAY = 110;   ///< Gris du fond.
const double RUNNER_LANE_WIDTH_RATIO = 0.5;  ///< Largeur de voie par defaut.

/// @brief Construit une image de voie synthetique.
::cv::Mat make_lane_frame( int p_left_x, int p_right_x )
  {
  const ::cv::Scalar background( RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY );
  ::cv::Mat frame( RUNNER_TEST_HEIGHT, RUNNER_TEST_WIDTH, CV_8UC3, background );
  const ::cv::Scalar line_color( 255, 255, 255 );
  const ::cv::Point left_bottom( p_left_x, RUNNER_TEST_HEIGHT - 1 );
  const ::cv::Point left_top( p_left_x, RUNNER_TEST_HEIGHT / 2 );
  const ::cv::Point right_bottom( p_right_x, RUNNER_TEST_HEIGHT - 1 );
  const ::cv::Point right_top( p_right_x, RUNNER_TEST_HEIGHT / 2 );
  ::cv::line( frame, left_bottom, left_top, line_color, RUNNER_LINE_THICKNESS );
  ::cv::line( frame, right_bottom, right_top, line_color, RUNNER_LINE_THICKNESS );
  return frame;
  }

/// @brief Source factice servant un nombre fixe de frames identiques.
class FakeFrameSource : public FrameSource
  {
  public:
    /// @brief Construit la source.
    /// @param p_frame_count Nombre de frames a servir.
    explicit FakeFrameSource( int p_frame_count )
      : m_remaining( p_frame_count )
      {
      }

    /// @brief Sert la frame suivante.
    bool read( ::cv::Mat& p_frame ) override
      {
      if ( 0 >= m_remaining )
        {
        return false;
        }

      p_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
      --m_remaining;
      return true;
      }

  private:
    int m_remaining;  ///< Frames restant a servir.
  };

/// @brief Observateur factice memorisant les index recus.
class FakeFrameObserver : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    FakeFrameObserver()
      : m_indices()
      {
      }

    /// @brief Memorise l'index recu.
    void on_frame( int p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   double p_compute_ms,
                   double p_render_ms ) override
      {
      (void) p_model;
      (void) p_compute_ms;
      (void) p_render_ms;
      const bool frame_is_empty = p_annotated_frame.empty();
      CHECK( false == frame_is_empty );
      m_indices.push_back( p_frame_index );
      }

    /// @brief Index recus, dans l'ordre.
    const ::std::vector< int >& indices() const
      {
      return m_indices;
      }

  private:
    ::std::vector< int > m_indices;  ///< Index recus.
  };

/// @brief Observateur factice signalant un echec definitif des la premiere frame.
class FatalAfterFirstFrameObserver : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    FatalAfterFirstFrameObserver()
      : m_frames_seen( 0 )
      {
      }

    /// @brief Compte la frame recue.
    void on_frame( int p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   double p_compute_ms,
                   double p_render_ms ) override
      {
      (void) p_frame_index;
      (void) p_model;
      (void) p_annotated_frame;
      (void) p_compute_ms;
      (void) p_render_ms;
      ++m_frames_seen;
      }

    /// @brief En echec des qu'au moins une frame a ete traitee.
    bool has_fatal_error() const override
      {
      const bool at_least_one_frame = ( 0 < m_frames_seen );
      return at_least_one_frame;
      }

  private:
    int m_frames_seen;  ///< Nombre de frames recues.
  };

/// @brief Observateur qui n'exploite pas l'image : memorise si elle etait vide.
class FrameAgnosticObserver : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    FrameAgnosticObserver()
      : m_all_frames_empty( true )
      {
      }

    /// @brief Memorise la vacuite de l'image recue.
    void on_frame( int p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   double p_compute_ms,
                   double p_render_ms ) override
      {
      (void) p_frame_index;
      (void) p_model;
      (void) p_compute_ms;
      (void) p_render_ms;
      const bool frame_is_empty = p_annotated_frame.empty();

      if ( !frame_is_empty )
        {
        m_all_frames_empty = false;
        }
      }

    /// @brief N'exploite pas l'image annotee.
    bool needs_annotated_frame() const override { return false; }

    /// @brief true si toutes les images recues etaient vides.
    bool all_frames_empty() const
      {
      return m_all_frames_empty;
      }

  private:
    bool m_all_frames_empty;  ///< true tant qu'aucune image non vide n'est recue.
  };

/// @brief Observateur qui exploite l'image : memorise sa taille.
class FrameHungryObserver : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    FrameHungryObserver()
      : m_last_size( 0, 0 )
      {
      }

    /// @brief Memorise la taille de l'image recue.
    void on_frame( int p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   double p_compute_ms,
                   double p_render_ms ) override
      {
      (void) p_frame_index;
      (void) p_model;
      (void) p_compute_ms;
      (void) p_render_ms;
      m_last_size = p_annotated_frame.size();
      }

    /// @brief Taille de la derniere image recue.
    ::cv::Size last_size() const
      {
      return m_last_size;
      }

  private:
    ::cv::Size m_last_size;  ///< Taille de la derniere image recue.
  };

} // namespace

TEST_CASE( "PipelineRunner : traite toutes les frames et compte les stats" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  // La source sert les frames APRES la premiere, passee directement au runner.
  FakeFrameSource source( RUNNER_TEST_FRAME_COUNT - 1 );
  FakeFrameObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  CHECK( EXIT_SUCCESS == status );
  CHECK( RUNNER_TEST_FRAME_COUNT == stats.frame_count );
  CHECK( RUNNER_TEST_FRAME_COUNT == static_cast< int >( observer.indices().size() ) );
  CHECK( 0 == observer.indices().front() );
  CHECK( ( RUNNER_TEST_FRAME_COUNT - 1 ) == observer.indices().back() );
  CHECK( 0.0 < stats.compute_ms );
  CHECK( stats.detected_count <= stats.frame_count );
}

TEST_CASE( "PipelineRunner : arret demande -> une seule frame traitee" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  FakeFrameSource source( RUNNER_TEST_FRAME_COUNT );
  FakeFrameObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( true );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  // La premiere frame est traitee, puis l'arret est constate avant la suivante.
  CHECK( EXIT_SUCCESS == status );
  CHECK( 1 == stats.frame_count );
}

TEST_CASE( "PipelineRunner : premiere frame vide -> echec" )
{
  const ::cv::Mat reference_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( reference_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  FakeFrameSource source( 0 );
  FakeFrameObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const ::cv::Mat empty_frame;
  const int status = runner.run( empty_frame, stats );

  CHECK( EXIT_FAILURE == status );
  CHECK( 0 == stats.frame_count );
}

TEST_CASE( "PipelineRunner : observateur en echec definitif -> arret anticipe" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  // La source pourrait servir davantage de frames que ce que le runner ne traite.
  FakeFrameSource source( RUNNER_TEST_FRAME_COUNT - 1 );
  FatalAfterFirstFrameObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  CHECK( EXIT_SUCCESS == status );
  CHECK( 1 == stats.frame_count );
}

TEST_CASE( "PipelineRunner : aucun observateur n'exploite l'image -> pas de rendu" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  FakeFrameSource source( RUNNER_TEST_FRAME_COUNT - 1 );
  FrameAgnosticObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  const bool all_frames_empty = observer.all_frames_empty();

  CHECK( EXIT_SUCCESS == status );
  CHECK( true == all_frames_empty );
  // Egalite stricte : le rendu n'a jamais ete execute, la duree cumulee est exactement nulle.
  CHECK( 0.0 == stats.render_ms );
  CHECK( 0.0 < stats.compute_ms );
}

TEST_CASE( "PipelineRunner : un observateur exploite l'image -> rendu execute" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );

  FakeFrameSource source( 0 );
  FrameHungryObserver observer;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &observer );
  const ::std::atomic< bool > stop_requested( false );

  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  const ::cv::Size received_size = observer.last_size();

  CHECK( EXIT_SUCCESS == status );
  CHECK( RUNNER_TEST_WIDTH == received_size.width );
  CHECK( RUNNER_TEST_HEIGHT == received_size.height );
}

TEST_CASE( "PipelineRunner : appelle le LaneTracker (coasting sur une frame sans detection)" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  config.lane_tracker_max_coast_frames = 5;
  // Trapeze BEV attendu par make_lane_frame (lignes de mi-hauteur au bas,
  // plein cadre) : la calibration par defaut de LaneConfig vise la camera
  // reelle et ne capte aucun pixel sur ce fixture synthetique (cf.
  // configure_synthetic_bev_trapezoid dans test_detect_lines.cpp).
  config.src_top_y_ratio = 0.45f;
  config.src_top_width_ratio = 0.18f;
  config.src_bottom_y_ratio = 1.0f;
  config.src_bottom_width_ratio = 0.5f;
  // Trapeze synthetique non representatif d'une vraie perspective camera :
  // seule cette calibration de test approximative ferait echouer le critere
  // de largeur de LaneQuality sur des lignes pourtant droites et paralleles.
  config.max_width_ratio_variation = 10.0;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );
  LaneTracker tracker( video, config, debug_sink );

  // Frame 0 : sans marquage (lane_detected=false). Frame 1 : avec marquages.
  class MixedDetectionFrameSource : public FrameSource
    {
    public:
      MixedDetectionFrameSource() : m_step( 0 ) { }

      bool read( ::cv::Mat& p_frame ) override
        {
        if ( 0 == m_step )
          {
          const ::cv::Scalar background( RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY );
          p_frame = ::cv::Mat( RUNNER_TEST_HEIGHT, RUNNER_TEST_WIDTH, CV_8UC3, background );
          ++m_step;
          return true;
          }

        if ( 1 == m_step )
          {
          p_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
          ++m_step;
          return true;
          }

        return false;
        }

    private:
      int m_step;
    };

  class ModelRecorder : public FrameObserver
    {
    public:
      ModelRecorder() : m_models() { }

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
        m_models.push_back( p_model );
        }

      bool needs_annotated_frame() const override { return false; }

      const ::std::vector< LaneModel >& models() const { return m_models; }

    private:
      ::std::vector< LaneModel > m_models;
    };

  MixedDetectionFrameSource source;
  ModelRecorder recorder;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &recorder );
  const ::std::atomic< bool > stop_requested( false );

  // Premiere frame (passee directement a run) : avec marquages -> detection fraiche.
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  CHECK( EXIT_SUCCESS == status );
  REQUIRE( 3 == static_cast< int >( recorder.models().size() ) );
  CHECK( recorder.models()[0].lane_detected );
  CHECK_FALSE( recorder.models()[0].coasted );
  CHECK( recorder.models()[1].lane_detected );   // frame sans marquage -> coasting
  CHECK( recorder.models()[1].coasted );
  CHECK( recorder.models()[2].lane_detected );   // detection fraiche a nouveau
  CHECK_FALSE( recorder.models()[2].coasted );
}
