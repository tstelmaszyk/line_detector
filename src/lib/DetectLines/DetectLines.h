#pragma once

/// @file
/// @brief Orchestration de la chaîne de détection de voie.

#include <opencv2/core.hpp>

#include "ImageSink/ImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneMask/LaneMask.h"
#include "LaneModel/LaneModel.h"
#include "LaneOverlay/LaneOverlay.h"
#include "LaneQuality/LaneQuality.h"
#include "PerspectiveView/PerspectiveView.h"
#include "SlidingWindowSearch/SlidingWindowSearch.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

/// @brief Orchestre : masque -> BEV -> fenêtres glissantes -> fit polynomial ->
/// géométrie -> overlay. Renvoie le LaneModel et dessine le résultat.
class DetectLines
  {
  public:
    /// @brief Construit le détecteur.
    /// @param p_video      Caractéristiques image.
    /// @param p_config     Configuration du pipeline.
    /// @param p_debug_sink Destination des traces de debug.
    DetectLines( const VideoCaracteristics& p_video,
                 const LaneConfig& p_config,
                 ImageSink& p_debug_sink );

    /// @brief Calcule le modèle de voie. Aucun rendu, aucune image de sortie.
    /// @param p_frame Image d'entrée BGR.
    /// @return Modèle de voie (signal de pilotage).
    LaneModel compute( const ::cv::Mat& p_frame ) const;

    /// @brief Dessine un modèle déjà calculé sur l'image d'origine.
    ///
    /// Le modèle n'est pas nécessairement celui de p_frame : un modèle lissé
    /// dans le temps peut être dessiné sur la frame courante.
    /// @param p_frame  Image d'origine BGR.
    /// @param p_model  Modèle à dessiner.
    /// @param p_output Image annotée produite.
    void render( const ::cv::Mat& p_frame,
                 const LaneModel& p_model,
                 ::cv::Mat& p_output ) const;

  private:
    const VideoCaracteristics m_video_properties;  ///< Caractéristiques image.
    const LaneConfig m_config;                      ///< Configuration du pipeline.
    ImageSink& m_debug_sink;                         ///< Destination des traces.
    LaneMask m_mask;                                 ///< Étape masque binaire.
    PerspectiveView m_perspective;                   ///< Étape BEV.
    SlidingWindowSearch m_search;                    ///< Étape fenêtres glissantes.
    LaneQuality m_quality;                            ///< Étape confiance (avant LaneGeometry).
    LaneOverlay m_overlay;                            ///< Étape rendu (déclarée après m_perspective).
  };
