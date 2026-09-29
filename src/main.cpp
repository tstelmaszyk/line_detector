/// @file
/// @brief Point d'entrée : assemble source, détecteur et observateurs, puis boucle.

#include <opencv2/core.hpp>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "CliOptions/CliOptions.h"
#include "DetectLines/DetectLines.h"
#include "FrameObserver/AnnotatedVideoWriter.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameObserver/LaneModelLogger.h"
#include "FrameObserver/ResultImageWriter.h"
#include "FrameSource/FrameSource.h"
#include "FrameSource/ImageFrameSource.h"
#include "FrameSource/VideoFrameSource.h"
#include "ImageSink/DiskImageSink.h"
#include "ImageSink/ImageSink.h"
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneTracker/LaneTracker.h"
#include "PipelineRunner/PipelineRunner.h"
#include "RunStats/RunStats.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

#include "SmartAssert/SmartAssert.h"

namespace
{

const char* const DEFAULT_OUTPUT_DIR = "out";                ///< Dossier de sortie par défaut.
const char* const OUTPUT_DIR_ENV_VAR = "LINE_DETECTOR_OUT";  ///< Variable d'env du dossier de sortie.
const char* const DEBUG_ENV_VAR = "LINE_DETECTOR_DEBUG";     ///< Variable d'env d'activation du debug.
const ::std::string OUTPUT_IMAGE_NAME = "output.jpg";        ///< Nom du fichier image résultat.
const ::std::string OUTPUT_VIDEO_NAME = "output.avi";        ///< Nom du fichier vidéo résultat.
const ::std::string PATH_SEPARATOR = "/";                    ///< Séparateur de chemin.
const double DEFAULT_LANE_WIDTH_RATIO = 0.35;                ///< Largeur de voie par défaut (fraction de W).
const double OUTPUT_VIDEO_FPS = 30.0;                        ///< Cadence déclarée de la vidéo de sortie.
const double MILLISECONDS_PER_SECOND = 1000.0;               ///< Conversion ms -> s.


/// @brief Drapeau d'arrêt levé par le handler SIGINT.
::std::atomic< bool > g_stop_requested( false );

/// @brief Handler SIGINT : demande l'arrêt de la boucle (fermeture propre des sorties).
///
/// Restaure d'abord l'action par défaut : un second Ctrl-C doit toujours pouvoir
/// terminer le processus, meme si VideoCapture::read est bloque sur une camera
/// debranchee et ne revient jamais tester g_stop_requested.
/// @param p_signal_number Numéro du signal reçu.
void handle_interrupt( int p_signal_number )
  {
  ::std::signal( SIGINT, SIG_DFL );
  (void) p_signal_number;
  g_stop_requested.store( true );
  }

/// @brief Construit la source de frames correspondant aux options.
///
/// La sous-commande (media_kind) choisit la classe, l'option de source
/// (source_kind) choisit la fabrique.
///
/// Les fabriques renvoient nullptr si la source ne peut pas être ouverte ; ce
/// nullptr est propagé tel quel, et main le traite comme un aléa
/// d'environnement (EXIT_IF_FAILED). Un objet non nul est toujours
/// utilisable : aucune autre vérification n'est nécessaire.
/// @param p_options Options de la ligne de commande (déjà validées par parse_arguments).
/// @return Source construite, ou nullptr si elle n'a pas pu être ouverte.
::std::unique_ptr< FrameSource > make_frame_source( const CliOptions& p_options )
  {
  if ( MEDIA_KIND_IMAGE == p_options.media_kind )
    {
    // parse_arguments rejette image --camera et image --gstreamer : seul --file arrive ici.
    const bool is_file_source = ( SOURCE_KIND_FILE == p_options.source_kind );
    SMART_ASSERT( is_file_source, "image n'accepte que --file" );
    return ImageFrameSource::from_file( p_options.input_path );
    }

  switch ( p_options.source_kind )
    {
    case SOURCE_KIND_FILE:
      return VideoFrameSource::from_file( p_options.input_path );

    case SOURCE_KIND_CAMERA:
      return VideoFrameSource::from_camera( p_options.camera_index );

    case SOURCE_KIND_GSTREAMER:
      return VideoFrameSource::from_gstreamer( p_options.gstreamer_pipeline );

    case SOURCE_KIND_COUNT:
      break;
    }

  SMART_ASSERT( false, "source_kind invalide" );
  return nullptr;
  }

} // namespace

/// @brief Point d'entrée : assemble la source, le détecteur et les observateurs,
/// puis délègue la boucle au PipelineRunner.
/// @param argc Nombre d'arguments.
/// @param argv Arguments de la ligne de commande.
/// @return EXIT_SUCCESS si le traitement s'est terminé normalement, EXIT_FAILURE sinon.
int main( int argc, char** argv )
{
  // 1. Analyse des arguments.
  CliOptions options;
  const int parse_status = parse_arguments( argc, argv, options );
  EXIT_IF_FAILED( EXIT_SUCCESS == parse_status, options.error_message + "\n" + USAGE_MESSAGE );

  // 2. Ouverture de la source de frames.
  ::std::unique_ptr< FrameSource > frame_source = make_frame_source( options );
  EXIT_IF_FAILED( nullptr != frame_source, "Impossible d'ouvrir la source demandee." );

  // 3. Premiere frame : elle definit la geometrie de tout le pipeline.
  ::cv::Mat first_frame;
  const bool first_read_ok = frame_source->read( first_frame );
  EXIT_IF_FAILED( first_read_ok, "Aucune frame lisible dans la source." );

  // 4. Dossier de sortie et traces de debug.
  const char* output_dir_env = ::std::getenv( OUTPUT_DIR_ENV_VAR );
  const ::std::string output_dir = ( nullptr != output_dir_env ) ? output_dir_env : DEFAULT_OUTPUT_DIR;

  const char* debug_env = ::std::getenv( DEBUG_ENV_VAR );
  const bool debug_enabled = ( nullptr != debug_env ) && ( '\0' != debug_env[0] );

  // Le dossier de sortie n'est necessaire que si quelque chose sera ecrit :
  // le resultat avec --record, ou les traces avec LINE_DETECTOR_DEBUG. Sans
  // cela, le programme n'ecrit aucun fichier et ne doit pas echouer sur un
  // dossier de sortie inutilisable (camera en tete, rootfs eventuellement
  // en lecture seule).
  const bool needs_output_dir = ( options.record || debug_enabled );

  if ( needs_output_dir )
    {
    // cv::imwrite ne cree pas le dossier de sortie : on le cree ici, avant tout sink.
    // Version non-levante : un dossier deja present n'est pas une erreur (create_directories
    // rend false sans lever), mais des droits insuffisants ou un fichier deja present a ce
    // chemin ne doivent pas terminer le process par une exception non rattrapee.
    ::std::error_code create_directory_error;
    ::std::filesystem::create_directories( output_dir, create_directory_error );

    const bool output_dir_usable = ::std::filesystem::is_directory( output_dir, create_directory_error );
    EXIT_IF_FAILED( output_dir_usable, "Impossible de creer ou d'utiliser le dossier de sortie : " + output_dir );
    }

  ::std::unique_ptr< ImageSink > debug_sink;

  if ( debug_enabled )
    {
    debug_sink = ::std::make_unique< DiskImageSink >( output_dir );
    }
  else
    {
    debug_sink = ::std::make_unique< NullImageSink >();
    }

  // 5. Detecteur.
  VideoCaracteristics video_properties( first_frame );

  LaneConfig config;
  // Largeur de voie par defaut (pixels BEV) pour reconstruire un cote manquant.
  config.default_lane_width_px =
    static_cast< double >( video_properties.width_pixel ) * DEFAULT_LANE_WIDTH_RATIO;

  const DetectLines detector( video_properties, config, *debug_sink );
  LaneTracker tracker( video_properties, config, *debug_sink );

  // 6. Observateurs : le log CSV est toujours present ; les ecritures ne le sont
  // que si --record est passe. Sans writer, aucun observateur ne reclame l'image
  // annotee et le rendu n'est jamais execute.
  LaneModelLogger logger( ::std::cout );

  ::std::vector< FrameObserver* > observers;
  observers.push_back( &logger );

  const bool is_still_image = ( MEDIA_KIND_IMAGE == options.media_kind );
  const ::std::string video_path = output_dir + PATH_SEPARATOR + OUTPUT_VIDEO_NAME;

  ::std::unique_ptr< DiskImageSink > result_sink;
  ::std::unique_ptr< ResultImageWriter > image_writer;
  ::std::unique_ptr< AnnotatedVideoWriter > video_writer;

  if ( options.record )
    {
    if ( is_still_image )
      {
      result_sink = ::std::make_unique< DiskImageSink >( output_dir );
      image_writer = ::std::make_unique< ResultImageWriter >( *result_sink, OUTPUT_IMAGE_NAME );
      observers.push_back( image_writer.get() );
      }
    else
      {
      video_writer = ::std::make_unique< AnnotatedVideoWriter >( video_path, OUTPUT_VIDEO_FPS );
      observers.push_back( video_writer.get() );
      }
    }

  // 7. Arret propre : sans cela, Ctrl-C laisse la video de sortie inexploitable.
  ::std::signal( SIGINT, handle_interrupt );

  // 8. Boucle.
  PipelineRunner runner( *frame_source, detector, tracker, observers, g_stop_requested );
  RunStats stats;
  const int run_status = runner.run( first_frame, stats );

  EXIT_IF_FAILED( EXIT_SUCCESS == run_status, "Aucune frame traitee." );

  // 9. Verification des sorties.
  const bool image_failed = ( nullptr != image_writer ) && image_writer->has_failed();
  const bool video_failed = ( nullptr != video_writer ) && video_writer->has_failed();

  EXIT_IF_FAILED( !image_failed,
                  "Impossible d'ecrire l'image de sortie : " + output_dir + PATH_SEPARATOR + OUTPUT_IMAGE_NAME );
  EXIT_IF_FAILED( !video_failed, "Impossible d'ecrire la video de sortie : " + video_path );

  // 10. Resume.
  const DurationMs total_ms = stats.compute_ms + stats.render_ms;
  const DurationMs average_ms = total_ms / static_cast< double >( stats.frame_count );
  const DurationMs average_render_ms = stats.render_ms / static_cast< double >( stats.frame_count );
  const double frames_per_second = MILLISECONDS_PER_SECOND / average_ms;

  ::std::cerr << " Frames : " << stats.frame_count
              << " | detectees : " << stats.detected_count
              << " | reconstruites : " << stats.reconstructed_count
              << " | moyenne : " << average_ms << " ms/frame"
              << " (dont " << average_render_ms << " ms de rendu)"
              << " (" << frames_per_second<< " FPS)" << ::std::endl;

  if ( options.record )
    {
    const ::std::string result_name = is_still_image ? OUTPUT_IMAGE_NAME : OUTPUT_VIDEO_NAME;
    ::std::cerr << "Resultat ecrit dans : " << output_dir << PATH_SEPARATOR
                << result_name << ::std::endl;
    }

  return EXIT_SUCCESS;
}
