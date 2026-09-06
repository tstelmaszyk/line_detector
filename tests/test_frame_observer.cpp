#include "doctest.h"

#include <opencv2/core.hpp>

#include <sstream>
#include <string>

#include "FrameObserver/AnnotatedVideoWriter.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameObserver/LaneModelLogger.h"
#include "FrameObserver/ResultImageWriter.h"
#include "ImageSink/ImageSink.h"
#include "LaneModel/LaneModel.h"
#include "test_support.h"

namespace
{

const int OBSERVER_TEST_WIDTH = 32;     ///< Largeur des frames de test.
const int OBSERVER_TEST_HEIGHT = 24;    ///< Hauteur des frames de test.
const double TEST_ELAPSED_MS = 12.5;    ///< Duree de traitement simulee.
const double TEST_LARGE_CURVATURE_RADIUS_PX = 1234567.0;  ///< Grand rayon (bascule en notation scientifique ?).

/// @brief Sink de test : memorise les appels au lieu d'ecrire sur disque.
class RecordingImageSink : public ImageSink
  {
  public:
    /// @brief Construit le sink avec le resultat a renvoyer.
    /// @param p_save_result Valeur rendue par save().
    explicit RecordingImageSink( bool p_save_result )
      : m_save_result( p_save_result ),
        m_save_count( 0 ),
        m_last_name()
      {
      }

    /// @brief Memorise l'appel.
    bool save( const ::std::string& p_name, const ::cv::Mat& p_frame ) override
      {
      (void) p_frame;
      ++m_save_count;
      m_last_name = p_name;
      return m_save_result;
      }

    /// @brief Nombre d'appels a save().
    int save_count() const
      {
      return m_save_count;
      }

    /// @brief Dernier nom de fichier recu.
    const ::std::string& last_name() const
      {
      return m_last_name;
      }

  private:
    bool m_save_result;         ///< Valeur rendue par save().
    int m_save_count;           ///< Nombre d'appels.
    ::std::string m_last_name;  ///< Dernier nom recu.
  };

/// @brief Construit un LaneModel de test.
LaneModel make_test_model( double p_normalized_offset )
  {
  LaneModel model;
  model.lane_detected = true;
  model.reconstructed = false;
  model.normalized_offset = p_normalized_offset;
  model.lateral_offset_px = 42.0;
  model.curvature_radius_px = 1500.0;
  return model;
  }

} // namespace

TEST_CASE( "LaneModelLogger : en-tete puis une ligne par frame" )
{
  ::std::ostringstream output;
  LaneModelLogger logger( output );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );

  const LaneModel first_model = make_test_model( -0.25 );
  const LaneModel second_model = make_test_model( 0.10 );
  logger.on_frame( 0, first_model, frame, TEST_ELAPSED_MS, 0.0 );
  logger.on_frame( 1, second_model, frame, TEST_ELAPSED_MS, 0.0 );

  const ::std::string text = output.str();

  // Une ligne d'en-tete + deux lignes de donnees.
  int line_count = 0;

  for ( const char character : text )
    {
    if ( '\n' == character )
      {
      ++line_count;
      }
    }

  CHECK( 3 == line_count );
  CHECK( ::std::string::npos != text.find( "frame_index" ) );
  CHECK( ::std::string::npos != text.find( "normalized_offset" ) );
  CHECK( ::std::string::npos != text.find( "-0.25" ) );
}

TEST_CASE( "LaneModelLogger : grand rayon de courbure -> pas de notation scientifique" )
{
  ::std::ostringstream output;
  LaneModelLogger logger( output );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );

  LaneModel model = make_test_model( 0.0 );
  model.curvature_radius_px = TEST_LARGE_CURVATURE_RADIUS_PX;
  logger.on_frame( 0, model, frame, TEST_ELAPSED_MS, 0.0 );

  const ::std::string text = output.str();
  // "e+"/"e-" est la marque de la notation scientifique ; le texte de l'en-tete
  // contient bien la lettre 'e' (compute_ms, render_ms), donc on cherche ce motif precis.
  const bool has_scientific_notation =
    ( ( ::std::string::npos != text.find( "e+" ) ) || ( ::std::string::npos != text.find( "e-" ) ) );

  CHECK( false == has_scientific_notation );
}

TEST_CASE( "ResultImageWriter : ecrit la frame annotee via l'ImageSink" )
{
  RecordingImageSink sink( true );
  ResultImageWriter writer( sink, "output.jpg" );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );
  const LaneModel model = make_test_model( 0.0 );

  writer.on_frame( 0, model, frame, TEST_ELAPSED_MS, 0.0 );

  const bool failed = writer.has_failed();

  CHECK( 1 == sink.save_count() );
  CHECK( ::std::string( "output.jpg" ) == sink.last_name() );
  CHECK( false == failed );
}

TEST_CASE( "ResultImageWriter : echec d'ecriture signale par has_failed" )
{
  RecordingImageSink sink( false );
  ResultImageWriter writer( sink, "output.jpg" );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );
  const LaneModel model = make_test_model( 0.0 );

  writer.on_frame( 0, model, frame, TEST_ELAPSED_MS, 0.0 );

  const bool failed = writer.has_failed();

  CHECK( true == failed );
}

TEST_CASE( "needs_annotated_frame : true par defaut, false pour le logger" )
{
  // Observateur nu : il herite du defaut de l'interface.
  class BareObserver : public FrameObserver
    {
    public:
      /// @brief Ne fait rien : seul le defaut de needs_annotated_frame est teste.
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
        }
    };

  BareObserver bare_observer;
  ::std::ostringstream output;
  LaneModelLogger logger( output );

  const bool bare_needs_frame = bare_observer.needs_annotated_frame();
  const bool logger_needs_frame = logger.needs_annotated_frame();

  CHECK( true == bare_needs_frame );
  CHECK( false == logger_needs_frame );
}

TEST_CASE( "needs_annotated_frame : true pour les deux writers" )
{
  RecordingImageSink sink( true );
  ResultImageWriter image_writer( sink, "output.jpg" );
  AnnotatedVideoWriter video_writer( test_temp_dir() + "/needs_frame.avi", 10.0 );

  const bool image_needs_frame = image_writer.needs_annotated_frame();
  const bool video_needs_frame = video_writer.needs_annotated_frame();

  CHECK( true == image_needs_frame );
  CHECK( true == video_needs_frame );
}

TEST_CASE( "LaneModelLogger : colonne coasted, entre reconstructed et compute_ms" )
{
  ::std::ostringstream output;
  LaneModelLogger logger( output );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );

  LaneModel model = make_test_model( 0.0 );
  model.coasted = true;
  logger.on_frame( 0, model, frame, TEST_ELAPSED_MS, 0.0 );

  const ::std::string text = output.str();

  CHECK( ::std::string::npos != text.find(
    "frame_index;lane_detected;normalized_offset;lateral_offset_px;"
    "curvature_radius_px;reconstructed;coasted;compute_ms;render_ms" ) );
  // reconstructed=0 (make_test_model), coasted=1 (force ci-dessus), compute_ms=12.500000.
  CHECK( ::std::string::npos != text.find(
    "0;1;0.000000;42.000000;1500.000000;0;1;12.500000;0.000000" ) );
}
