#include "doctest.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/registry.hpp>

#include <fstream>
#include <memory>
#include <string>

#include "FrameSource/ImageFrameSource.h"
#include "FrameSource/VideoFrameSource.h"
#include "test_support.h"

namespace
{

const int TEST_IMAGE_WIDTH = 64;    ///< Largeur des images de test.
const int TEST_IMAGE_HEIGHT = 48;   ///< Hauteur des images de test.
const int TEST_GRAY_LEVEL = 120;    ///< Niveau de gris de remplissage.

const int TEST_VIDEO_FRAME_COUNT = 5;         ///< Frames ecrites dans la video de test.
const double TEST_VIDEO_FPS = 10.0;           ///< Cadence de la video de test.
const int TEST_VIDEO_GRAY_STEP = 20;          ///< Ecart de gris entre deux frames.

/// @brief Ecrit une image de test sur disque et rend son chemin.
::std::string write_test_image( const ::std::string& p_file_name )
  {
  const ::std::string full_path = test_temp_dir() + "/" + p_file_name;
  const ::cv::Scalar fill_color( TEST_GRAY_LEVEL, TEST_GRAY_LEVEL, TEST_GRAY_LEVEL );
  const ::cv::Mat image( TEST_IMAGE_HEIGHT, TEST_IMAGE_WIDTH, CV_8UC3, fill_color );
  const bool written = ::cv::imwrite( full_path, image );
  REQUIRE( written );
  return full_path;
  }

/// @brief Ecrit une petite video de test sur disque et rend son chemin.
::std::string write_test_video( const ::std::string& p_file_name )
  {
  const ::std::string full_path = test_temp_dir() + "/" + p_file_name;
  const int fourcc = ::cv::VideoWriter::fourcc( 'M', 'J', 'P', 'G' );
  const ::cv::Size frame_size( TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT );
  ::cv::VideoWriter writer( full_path, fourcc, TEST_VIDEO_FPS, frame_size );
  const bool opened = writer.isOpened();
  REQUIRE( true == opened );

  for ( int frame_index = 0; frame_index < TEST_VIDEO_FRAME_COUNT; ++frame_index )
    {
    const int gray_level = TEST_GRAY_LEVEL + ( frame_index * TEST_VIDEO_GRAY_STEP );
    const ::cv::Scalar fill_color( gray_level, gray_level, gray_level );
    const ::cv::Mat frame( TEST_IMAGE_HEIGHT, TEST_IMAGE_WIDTH, CV_8UC3, fill_color );
    writer.write( frame );
    }

  writer.release();
  return full_path;
  }

/// @brief Ecrit un fichier texte (ni image ni video) et rend son chemin.
::std::string write_text_file( const ::std::string& p_file_name )
  {
  const ::std::string full_path = test_temp_dir() + "/" + p_file_name;
  ::std::ofstream stream( full_path );
  stream << "ceci n'est pas une image\n";
  stream.close();
  const bool written = !stream.fail();
  REQUIRE( written );
  return full_path;
  }

const int GSTREAMER_TEST_FRAME_COUNT = 5;  ///< Frames produites par videotestsrc (num-buffers).

/// @brief Partie source d'un pipeline de test, sans caméra : 5 frames 64x48.
const ::std::string GSTREAMER_TEST_SOURCE =
  "videotestsrc num-buffers=" + ::std::to_string( GSTREAMER_TEST_FRAME_COUNT )
  + " ! video/x-raw,width=" + ::std::to_string( TEST_IMAGE_WIDTH )
  + ",height=" + ::std::to_string( TEST_IMAGE_HEIGHT );

/// @brief Exige le backend GStreamer d'OpenCV : échec explicite s'il manque, jamais ignoré.
void require_gstreamer_backend()
  {
  const bool has_gstreamer = ::cv::videoio_registry::hasBackend( ::cv::CAP_GSTREAMER );
  INFO( "Backend GStreamer absent d'OpenCV : installer un OpenCV compile avec GStreamer "
        "et le paquet gstreamer1.0-plugins-base (image Docker ou Pi)." );
  REQUIRE( has_gstreamer );
  }

} // namespace

TEST_CASE( "ImageFrameSource : une frame puis fin de flux" )
{
  const ::std::string image_path = write_test_image( "image_source.jpg" );
  const ::std::unique_ptr< ImageFrameSource > source = ImageFrameSource::from_file( image_path );
  REQUIRE( nullptr != source );

  ::cv::Mat frame;
  const bool first_read = source->read( frame );
  const bool second_read = source->read( frame );

  CHECK( true == first_read );
  CHECK( TEST_IMAGE_WIDTH == frame.cols );
  CHECK( TEST_IMAGE_HEIGHT == frame.rows );
  CHECK( CV_8UC3 == frame.type() );
  CHECK( false == second_read );
}

TEST_CASE( "ImageFrameSource : fichier absent -> nullptr" )
{
  const ::std::string missing_path = test_temp_dir() + "/fichier_absent_line_detector.jpg";

  const ::std::unique_ptr< ImageFrameSource > source = ImageFrameSource::from_file( missing_path );

  CHECK( nullptr == source );
}

TEST_CASE( "ImageFrameSource : fichier qui n'est pas une image -> nullptr" )
{
  const ::std::string text_path = write_text_file( "pas_une_image_line_detector.jpg" );

  const ::std::unique_ptr< ImageFrameSource > source = ImageFrameSource::from_file( text_path );

  CHECK( nullptr == source );
}

TEST_CASE( "VideoFrameSource : lit toutes les frames d'un fichier video" )
{
  const ::std::string video_path = write_test_video( "video_source.avi" );
  const ::std::unique_ptr< VideoFrameSource > source = VideoFrameSource::from_file( video_path );
  REQUIRE( nullptr != source );

  int read_count = 0;
  ::cv::Mat frame;
  bool read_ok = source->read( frame );

  while ( read_ok )
    {
    ++read_count;
    CHECK( TEST_IMAGE_WIDTH == frame.cols );
    CHECK( TEST_IMAGE_HEIGHT == frame.rows );
    read_ok = source->read( frame );
    }

  CHECK( TEST_VIDEO_FRAME_COUNT == read_count );
}

TEST_CASE( "VideoFrameSource : fichier absent -> nullptr" )
{
  const ::std::string missing_path = test_temp_dir() + "/video_absente_line_detector.avi";

  const ::std::unique_ptr< VideoFrameSource > source = VideoFrameSource::from_file( missing_path );

  CHECK( nullptr == source );
}

TEST_CASE( "VideoFrameSource::from_gstreamer : frames BGR puis fin de flux" )
{
  require_gstreamer_backend();

  const ::std::unique_ptr< VideoFrameSource > source = VideoFrameSource::from_gstreamer( GSTREAMER_TEST_SOURCE );
  REQUIRE( nullptr != source );

  int read_count = 0;
  ::cv::Mat frame;
  bool read_ok = source->read( frame );

  while ( read_ok )
    {
    ++read_count;
    CHECK( TEST_IMAGE_WIDTH == frame.cols );
    CHECK( TEST_IMAGE_HEIGHT == frame.rows );
    CHECK( CV_8UC3 == frame.type() );
    read_ok = source->read( frame );
    }

  CHECK( GSTREAMER_TEST_FRAME_COUNT == read_count );
}

TEST_CASE( "VideoFrameSource::from_gstreamer : espaces autour de la source acceptes" )
{
  require_gstreamer_backend();

  const ::std::string padded_source = "  " + GSTREAMER_TEST_SOURCE + "  ";
  const ::std::unique_ptr< VideoFrameSource > source = VideoFrameSource::from_gstreamer( padded_source );
  REQUIRE( nullptr != source );

  ::cv::Mat frame;
  const bool read_ok = source->read( frame );

  CHECK( true == read_ok );
  CHECK( CV_8UC3 == frame.type() );
}

TEST_CASE( "VideoFrameSource::from_gstreamer : element inconnu au milieu -> nullptr" )
{
  require_gstreamer_backend();

  // Element inconnu au milieu uniquement : un premier element inconnu bloque
  // l'ouverture dans OpenCV (limite connue, cf. spec), il n'est donc pas teste.
  const ::std::string broken_source = "videotestsrc num-buffers=5 ! nexistepas_line_detector";

  const ::std::unique_ptr< VideoFrameSource > source = VideoFrameSource::from_gstreamer( broken_source );

  CHECK( nullptr == source );
}
