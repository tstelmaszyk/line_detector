#pragma once

/// @file
/// @brief Source de frames adossée à cv::VideoCapture (fichier vidéo, caméra).

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <memory>
#include <string>

#include "FrameSource/FrameSource.h"
#include "projectTypes.h"

/// @brief Source d'un flux de frames lu jusqu'à sa fin.
///
/// Une seule classe pour toutes les provenances : cv::VideoCapture les traite
/// de la même façon, seule la donnée passée à l'ouverture change. La
/// sous-commande `video` choisit cette classe ; l'option de source choisit la
/// fabrique.
///
/// Contrat d'échec : les fabriques renvoient nullptr si la source ne peut pas
/// être ouverte. Un objet reçu est donc toujours utilisable, sans autre
/// vérification (il n'y a volontairement pas de is_opened()).
class VideoFrameSource final : public FrameSource
  {
  public:
    /// @brief Fabrique une source lisant un fichier vidéo.
    /// @param p_video_path Chemin du fichier vidéo.
    /// @return Source ouverte, ou nullptr si le fichier est absent ou illisible.
    static ::std::unique_ptr< VideoFrameSource > from_file( const ::std::string& p_video_path );

    /// @brief Fabrique une source lisant une caméra (V4L2 sous Linux).
    /// @param p_camera_index Index de la caméra.
    /// @return Source ouverte, ou nullptr si la caméra est indisponible.
    static ::std::unique_ptr< VideoFrameSource > from_camera( CameraIndex p_camera_index );

    /// @brief Fabrique une source lisant un pipeline GStreamer.
    ///
    /// L'appelant ne fournit que la partie source (par exemple
    /// "libcamerasrc ! video/x-raw,width=1280,height=720") ; la fabrique
    /// ajoute la fin du pipeline (conversion BGR et appsink), qui est une
    /// exigence du programme et non un choix de l'utilisateur.
    ///
    /// Limite connue (OpenCV 4.6) : si le premier élément n'existe pas,
    /// l'ouverture ne rend jamais la main. Un élément inconnu ailleurs dans le
    /// pipeline donne nullptr.
    /// @param p_source_pipeline Partie source du pipeline : non blanche, sans appsink.
    /// @return Source ouverte, ou nullptr si le pipeline ne peut pas être
    ///         démarré (élément inconnu, backend GStreamer absent d'OpenCV).
    static ::std::unique_ptr< VideoFrameSource > from_gstreamer( const ::std::string& p_source_pipeline );

    /// @brief Non copiable : possède une cv::VideoCapture ouverte sur une ressource.
    VideoFrameSource( const VideoFrameSource& p_other ) = delete;

    /// @brief Non copiable : possède une cv::VideoCapture ouverte sur une ressource.
    VideoFrameSource& operator=( const VideoFrameSource& p_other ) = delete;

    /// @brief Lit la frame suivante.
    /// @param p_frame Frame lue (BGR).
    /// @return true si une frame a été lue, false à la fin du flux.
    bool read( ::cv::Mat& p_frame ) override;

  private:
    /// @brief Construit une source non ouverte (usage réservé aux fabriques).
    VideoFrameSource();

    /// @brief Applique le contrat d'échec des fabriques.
    /// @param p_source Source dont l'ouverture vient d'être tentée.
    /// @return p_source si la capture est ouverte, nullptr sinon.
    static ::std::unique_ptr< VideoFrameSource > keep_if_opened( ::std::unique_ptr< VideoFrameSource > p_source );

    ::cv::VideoCapture m_capture;  ///< Capture OpenCV sous-jacente, toujours ouverte hors fabriques.
  };
