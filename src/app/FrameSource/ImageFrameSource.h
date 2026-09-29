#pragma once

/// @file
/// @brief Source de frames : une image fixe, rendue une seule fois.

#include <opencv2/core.hpp>

#include <memory>
#include <string>

#include "FrameSource/FrameSource.h"

/// @brief Source d'une image fixe : rend la frame une fois, puis la fin de flux.
///
/// Fait du mode image fixe un cas dégénéré du mode vidéo : un seul chemin de
/// code dans la boucle applicative. La sous-commande `image` choisit cette
/// classe ; l'option de source choisit la fabrique (seul `--file` existe pour
/// l'instant).
///
/// Contrat d'échec : les fabriques renvoient nullptr si l'image ne peut pas
/// être obtenue. Un objet reçu est donc toujours utilisable, sans autre
/// vérification (il n'y a volontairement pas de is_opened()).
class ImageFrameSource final : public FrameSource
  {
  public:
    /// @brief Fabrique une source à partir d'un fichier image (cv::imread).
    /// @param p_image_path Chemin de l'image à charger.
    /// @return Source prête à l'emploi, ou nullptr si le fichier est absent ou
    ///         ne peut pas être décodé comme une image.
    static ::std::unique_ptr< ImageFrameSource > from_file( const ::std::string& p_image_path );

    /// @brief Rend l'image au premier appel, la fin de flux ensuite.
    /// @param p_frame Frame lue (BGR) ; non modifiée à la fin du flux.
    /// @return true au premier appel, false ensuite.
    bool read( ::cv::Mat& p_frame ) override;

  private:
    /// @brief Construit la source autour d'une image déjà chargée (usage réservé aux fabriques).
    /// @param p_image Image chargée, non vide.
    explicit ImageFrameSource( const ::cv::Mat& p_image );

    ::cv::Mat m_image;           ///< Image chargée par la fabrique, jamais vide.
    bool m_already_delivered;    ///< true une fois la frame rendue.
  };
