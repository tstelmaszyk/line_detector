#pragma once

/// @file
/// @brief Lissage temporel du LaneModel a travers les frames (EMA + coasting).

#include <opencv2/core.hpp>

#include "ImageSink/ImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneModel/LaneModel.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

/// @brief Lisse les polynomes de voie a travers le temps (moyenne mobile
/// exponentielle par coefficient), et tolere une perte de detection courte en
/// reconduisant le dernier modele lisse connu (coasting). Seul composant de
/// line_detector_lib qui porte un etat entre deux appels : deliberement non
/// possede par DetectLines (qui reste sans etat, cf. PipelineRunner.h), mais
/// construit et possede par PipelineRunner.
class LaneTracker
  {
  public:
    /// @brief Construit le tracker.
    /// @param p_video      Caracteristiques image (= taille BEV, cf. PerspectiveView::bev_size).
    /// @param p_config     Configuration du pipeline.
    /// @param p_debug_sink Destination des traces de debug.
    LaneTracker( const VideoCaracteristics& p_video,
                const LaneConfig& p_config,
                ImageSink& p_debug_sink );

    /// @brief Lisse un LaneModel brut a travers le temps.
    /// @param p_raw_model Modele de la frame courante (sortie de DetectLines::compute).
    /// @return Modele lisse ; p_raw_model tel quel si le tracker est desactive
    /// par configuration, ou si aucun etat exploitable n'existe encore.
    LaneModel update( const LaneModel& p_raw_model );

    /// @brief Reinitialise l'etat interne. Jamais appele par PipelineRunner en
    /// usage normal (une instance vit le temps d'un run) ; utilitaire de test.
    void reset();

  private:
    /// @brief Ecrit debug_04c_tracker.jpg : polynomes bruts vs lisses.
    /// @param p_raw_model Modele brut de la frame courante (pour les courbes brutes).
    /// @param p_coasted   true si cette frame est en coasting.
    void draw_debug_trace( const LaneModel& p_raw_model, bool p_coasted ) const;

    const VideoCaracteristics m_video_properties;  ///< Caracteristiques image (= taille BEV).
    const LaneConfig m_config;                      ///< Configuration du pipeline.
    ImageSink& m_debug_sink;                         ///< Destination des traces.

    LaneModel m_smoothed_model;  ///< Dernier modele lisse complet (polynomes + signal deja recalcule).
    bool m_has_state;            ///< true des qu'un premier fit valide a ete vu.
    int m_miss_streak;           ///< Detections manquees consecutives.
  };
