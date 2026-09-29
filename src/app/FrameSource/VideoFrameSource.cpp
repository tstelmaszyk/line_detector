/// @file
/// @brief Implémentation de VideoFrameSource.

#include <memory>
#include <string>
#include <utility>

#include "FrameSource/VideoFrameSource.h"
#include "SmartAssert/SmartAssert.h"

namespace
{

/// @brief Fin de pipeline ajoutée par from_gstreamer à la partie source fournie.
///
/// - videoconvert + format=BGR : le détecteur exige du BGR ; l'imposer ici évite
///   qu'un oubli de l'utilisateur produise un échec obscur.
/// - appsink drop=true max-buffers=1 : en direct, garde toujours la frame la plus
///   récente ; sans cela, un traitement plus lent que la source accumule un
///   retard qui grandit indéfiniment.
const ::std::string GSTREAMER_PIPELINE_TAIL =
  " ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";

} // namespace

VideoFrameSource::VideoFrameSource()
  : m_capture()
{
}

::std::unique_ptr< VideoFrameSource > VideoFrameSource::keep_if_opened( ::std::unique_ptr< VideoFrameSource > p_source )
{
  const bool opened = p_source->m_capture.isOpened();

  // Contrat d'échec : une capture non ouverte n'est jamais rendue à l'appelant.
  if ( !opened )
    {
    return nullptr;
    }

  return p_source;
}

::std::unique_ptr< VideoFrameSource > VideoFrameSource::from_file( const ::std::string& p_video_path )
{
  ::std::unique_ptr< VideoFrameSource > source( new VideoFrameSource() );
  source->m_capture.open( p_video_path );
  return keep_if_opened( ::std::move( source ) );
}

::std::unique_ptr< VideoFrameSource > VideoFrameSource::from_camera( CameraIndex p_camera_index )
{
  ::std::unique_ptr< VideoFrameSource > source( new VideoFrameSource() );
  source->m_capture.open( p_camera_index );
  return keep_if_opened( ::std::move( source ) );
}

::std::unique_ptr< VideoFrameSource > VideoFrameSource::from_gstreamer( const ::std::string& p_source_pipeline )
{
  // Précondition garantie par parse_arguments : une source vide bloquerait l'ouverture.
  const bool is_empty = p_source_pipeline.empty();
  SMART_ASSERT( !is_empty, "from_gstreamer appele avec une partie source vide" );

  const ::std::string full_pipeline = p_source_pipeline + GSTREAMER_PIPELINE_TAIL;

  ::std::unique_ptr< VideoFrameSource > source( new VideoFrameSource() );
  source->m_capture.open( full_pipeline, ::cv::CAP_GSTREAMER );
  return keep_if_opened( ::std::move( source ) );
}

bool VideoFrameSource::read( ::cv::Mat& p_frame )
{
  const bool read_ok = m_capture.read( p_frame );
  return read_ok;
}
