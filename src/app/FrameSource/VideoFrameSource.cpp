/// @file
/// @brief Implémentation de VideoFrameSource.

#include <memory>
#include <string>
#include <utility>

#include "FrameSource/VideoFrameSource.h"

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

bool VideoFrameSource::read( ::cv::Mat& p_frame )
{
  const bool read_ok = m_capture.read( p_frame );
  return read_ok;
}
