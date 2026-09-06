#pragma once

/// @file
/// @brief Résultat de la détection : modèle de voie + signal de pilotage.

#include "LanePolynomial/LanePolynomial.h"
#include "projectTypes.h"

/// @brief Résultat complet : deux polynômes de voie (pixels BEV) + signal de
/// pilotage rempli par LaneGeometry. Consommé plus tard par le module de contrôle.
struct LaneModel
  {
  LanePolynomial left;               ///< Polynôme du côté gauche (pixels BEV).
  LanePolynomial right;              ///< Polynôme du côté droit (pixels BEV).
  bool lane_detected = false;        ///< true si une voie exploitable est détectée.
  bool reconstructed = false;        ///< true si un côté a été reconstruit (signal dégradé).
  bool coasted = false;              ///< true si ce frame n'a pas de detection fraiche (LaneTracker reconduit le dernier modele lisse).

  PixelOffset lateral_offset_px = 0.0;  ///< Écart véhicule↔centre voie au bas de l'image.
  double normalized_offset = 0.0;    ///< offset / demi-largeur ; <0 = décalé à gauche.
  double curvature_radius_px = 0.0;  ///< Rayon de courbure (grand = quasi droit).
  };
