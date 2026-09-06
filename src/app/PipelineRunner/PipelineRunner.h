#pragma once

/// @file
/// @brief Boucle applicative : lit des frames, détecte, notifie les observateurs.

#include <opencv2/core.hpp>

#include <atomic>
#include <vector>

#include "DetectLines/DetectLines.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameSource/FrameSource.h"
#include "LaneTracker/LaneTracker.h"
#include "RunStats/RunStats.h"
#include "projectTypes.h"

/// @brief Possède la boucle de traitement d'un flux de frames.
///
/// Ne construit ni la source, ni le détecteur, ni les observateurs : ils lui
/// sont fournis. C'est le seul endroit du projet qui itère sur des frames — la
/// bibliothèque de détection reste sans état, appelable frame par frame.
class PipelineRunner
  {
  public:
    /// @brief Construit le runner.
    /// @param p_frame_source    Source de frames.
    /// @param p_detector        Pipeline de détection.
    /// @param p_tracker         Lissage temporel du LaneModel entre compute() et render().
    /// @param p_observers       Observateurs notifiés à chaque frame.
    /// @param p_stop_requested  Drapeau d'arrêt (levé par le handler SIGINT).
    PipelineRunner( FrameSource& p_frame_source,
                    const DetectLines& p_detector,
                    LaneTracker& p_tracker,
                    const ::std::vector< FrameObserver* >& p_observers,
                    const ::std::atomic< bool >& p_stop_requested );

    /// @brief Exécute la boucle jusqu'à la fin du flux, l'arrêt demandé, ou
    /// l'échec définitif d'un observateur.
    /// @param p_first_frame Première frame, déjà lue par l'appelant.
    /// @param p_stats       Statistiques accumulées par l'appel (struct fraîche attendue).
    /// @return EXIT_SUCCESS si au moins une frame a été traitée, EXIT_FAILURE sinon.
    int run( const ::cv::Mat& p_first_frame, RunStats& p_stats );

  private:
    /// @brief Traite une frame et notifie les observateurs.
    /// @param p_frame       Frame à traiter.
    /// @param p_frame_index Index de la frame.
    /// @param p_stats       Statistiques mises à jour.
    /// @return true si un observateur signale un échec définitif après cette frame.
    bool process_frame( const ::cv::Mat& p_frame, FrameIndex p_frame_index, RunStats& p_stats );

    FrameSource& m_frame_source;                      ///< Source de frames.
    const DetectLines& m_detector;                    ///< Pipeline de détection.
    LaneTracker& m_tracker;                            ///< Lissage temporel du LaneModel.
    ::std::vector< FrameObserver* > m_observers;      ///< Observateurs notifiés.
    const ::std::atomic< bool >& m_stop_requested;    ///< Drapeau d'arrêt.
    bool m_render_needed;                             ///< true si au moins un observateur exploite l'image annotée.
  };
