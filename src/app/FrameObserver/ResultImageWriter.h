#pragma once

/// @file
/// @brief Observateur écrivant la frame annotée en image (mode image fixe).

#include <opencv2/core.hpp>

#include <string>

#include "FrameObserver/FrameObserver.h"
#include "ImageSink/ImageSink.h"
#include "projectTypes.h"

struct LaneModel;

/// @brief Écrit la frame annotée via un ImageSink.
///
/// Utilisé par la sous-commande image : conserve à l'identique le comportement historique
/// (out/output.jpg écrit par un ImageSink injecté).
class ResultImageWriter : public FrameObserver
  {
  public:
    /// @brief Construit l'observateur.
    /// @param p_image_sink Destination d'écriture.
    /// @param p_file_name  Nom du fichier résultat.
    ResultImageWriter( ImageSink& p_image_sink, ::std::string p_file_name );

    /// @brief Écrit la frame annotée.
    /// @param p_frame_index     Index de la frame (inutilisé ici).
    /// @param p_model           Modèle de voie (inutilisé ici).
    /// @param p_annotated_frame Image annotée à écrire.
    /// @param p_compute_ms      Durée du calcul (inutilisée ici).
    /// @param p_render_ms       Durée du rendu (inutilisée ici).
    void on_frame( FrameIndex p_frame_index,
                   const LaneModel& p_model,
                   const ::cv::Mat& p_annotated_frame,
                   DurationMs p_compute_ms,
                   DurationMs p_render_ms ) override;

    /// @brief Indique si une écriture a échoué.
    /// @return true si au moins une écriture a échoué.
    bool has_failed() const;

    /// @brief Indique si l'écriture est dans un état d'échec définitif.
    /// @return has_failed().
    bool has_fatal_error() const override { return has_failed(); }

    /// @brief L'écriture du résultat exploite l'image annotée.
    /// @return true.
    bool needs_annotated_frame() const override { return true; }

  private:
    ImageSink& m_image_sink;      ///< Destination d'écriture.
    ::std::string m_file_name;    ///< Nom du fichier résultat.
    bool m_has_failed;            ///< true si une écriture a échoué.
  };
