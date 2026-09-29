/// @file
/// @brief Implémentation de ImageFrameSource.

#include <opencv2/imgcodecs.hpp>

#include <memory>
#include <string>

#include "FrameSource/ImageFrameSource.h"
#include "SmartAssert/SmartAssert.h"

ImageFrameSource::ImageFrameSource( const ::cv::Mat& p_image )
  : m_image( p_image ),
    m_already_delivered( false )
{
  const bool is_empty = m_image.empty();
  SMART_ASSERT( !is_empty, "ImageFrameSource construite sur une image vide" );
}

::std::unique_ptr< ImageFrameSource > ImageFrameSource::from_file( const ::std::string& p_image_path )
{
  const ::cv::Mat image = ::cv::imread( p_image_path, ::cv::IMREAD_COLOR );
  const bool is_empty = image.empty();

  // Contrat d'échec : image absente ou illisible -> nullptr, jamais un objet inutilisable.
  if ( is_empty )
    {
    return nullptr;
    }

  ::std::unique_ptr< ImageFrameSource > source( new ImageFrameSource( image ) );
  return source;
}

bool ImageFrameSource::read( ::cv::Mat& p_frame )
{
  if ( m_already_delivered )
    {
    return false;
    }

  p_frame = m_image.clone();
  m_already_delivered = true;
  return true;
}
