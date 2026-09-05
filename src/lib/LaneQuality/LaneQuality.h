#pragma once

/// @file
/// @brief Vérification de confiance des fits avant le calcul du signal de pilotage.

#include <opencv2/core.hpp>

#include <string>

#include "ImageSink/ImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneModel/LaneModel.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

/// @brief Démote (`LanePolynomial::valid = false`) les côtés dont le fit n'est
/// pas digne de confiance, avant que `LaneGeometry` ne calcule le signal de
/// pilotage : pixels insuffisants, ou largeur de voie incohérente entre le
/// bas, le milieu et le haut de la BEV (lignes qui divergent ou se croisent —
/// cas d'un mauvais angle caméra). Pur aléa de la route -> drapeau, jamais de
/// SMART_ASSERT. N'ajoute aucun champ à LaneModel : le rejet reste visible
/// uniquement via `LaneModel::lane_detected` (calculé ensuite par
/// LaneGeometry sur les côtés démotés).
class LaneQuality
  {
  public:
    /// @brief Construit le vérificateur.
    /// @param p_video      Caractéristiques image.
    /// @param p_config     Configuration du pipeline.
    /// @param p_debug_sink Destination des traces de debug.
    LaneQuality( const VideoCaracteristics& p_video,
                const LaneConfig& p_config,
                ImageSink& p_debug_sink );

    /// @brief Démote les côtés non dignes de confiance.
    /// @param p_bev   Image BEV binaire (CV_8UC1) ayant produit les pixels du
    ///                fit — utilisée uniquement pour la trace de debug.
    /// @param p_model Modèle juste après LanePolynomial::fit (avant LaneGeometry).
    /// @return Modèle avec les côtés non fiables démotés (valid = false).
    LaneModel evaluate( const ::cv::Mat& p_bev, LaneModel p_model ) const;

  private:
    /// @brief Écrit debug_04b_quality.jpg avec les fits d'origine et la raison.
    void draw_debug_trace( const ::cv::Mat& p_bev,
                           const LanePolynomial& p_left,
                           const LanePolynomial& p_right,
                           const ::std::string& p_reason ) const;

    const VideoCaracteristics m_video_properties;  ///< Caractéristiques image.
    const LaneConfig m_config;                      ///< Configuration du pipeline.
    ImageSink& m_debug_sink;                         ///< Destination des traces.
  };
