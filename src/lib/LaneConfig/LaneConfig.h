#pragma once

/// @file
/// @brief Configuration centralisée de la chaîne de détection de voie.

#include <opencv2/core.hpp>

#include "projectTypes.h"

/// @brief Regroupe tous les réglages du pipeline (seuillage, calibration BEV,
/// fenêtres glissantes). Injecté par valeur const dans chaque composant. Les
/// valeurs par défaut ci-dessous sont la définition centralisée des constantes
/// de réglage (cf. CLAUDE.md).
struct LaneConfig
  {
  // --- Seuillage (LaneMask) ---
  int white_threshold = 200;              ///< Luminance mini du marquage blanc.
  ::cv::Vec2i yellow_hue = { 15, 35 };    ///< Plage de teinte jaune (HSV).
  int blur_kernel = 5;                    ///< Flou avant Sobel (impair) : tue la texture asphalte.
  int sobel_kernel = 3;                   ///< Taille du noyau Sobel.
  int sobel_thresh_low = 80;              ///< Seuil bas du gradient Sobel.
  int sobel_thresh_high = 150;            ///< Seuil haut du gradient Sobel.
  float sobel_near_cutoff_ratio = 0.6f;   ///< Sobel ignoré sous cette fraction de H (champ proche bruité).
  int morph_kernel = 3;                   ///< Ouverture morpho du masque final (impair).

  // --- BEV (PerspectiveView) ---
  // TODO(calibration) : ces 4 ratios sont calibres a la main pour la camera du
  // vehicule (montage tres bas, ~5-6 cm du sol, mesures sur img_piste/koimg.png).
  // Ce sont des constantes de compilation alors que c'est un reglage propre a
  // CHAQUE camera/montage : trouver un mecanisme de calibration qui ne force pas
  // a recompiler (variable d'environnement sur le modele de LINE_DETECTOR_OUT,
  // fichier de config, ou flag CLI) plutot que de coder en dur ces valeurs ici.
  float src_top_width_ratio = 0.35f;      ///< Demi-largeur du bord haut du quad (fraction de W).
  float src_top_y_ratio = 0.10f;          ///< Hauteur du bord haut (fraction de H).
  float src_bottom_width_ratio = 0.44f;   ///< Demi-largeur du bord bas du quad (fraction de W). 0.5 = plein cadre.
  float src_bottom_y_ratio = 0.30f;       ///< Hauteur du bord bas (fraction de H). 1.0 = bas de l'image.
  float bev_margin_ratio = 0.20f;         ///< Marge latérale du rectangle BEV (fraction de W).

  // --- Fenêtres glissantes (SlidingWindowSearch) ---
  int window_count = 9;                   ///< Nombre de fenêtres glissantes.
  int window_margin = 60;                 ///< Demi-largeur d'une fenêtre (pixels).
  int first_window_margin = 100;          ///< Marge élargie de la 1re fenêtre (bas).
  int window_min_pix = 50;                ///< Pixels mini pour recentrer une fenêtre.
  float histogram_band_ratio = 0.25f;     ///< Fraction basse de H pour l'histogramme de base.

  // --- Reconstruction / sanité (LaneGeometry) ---
  PixelOffset default_lane_width_px = 0.0;  ///< 0 = pas de reconstruction du côté manquant.

  // --- Qualité / confiance (LaneQuality) ---
  int min_quality_points = 150;              ///< Pixels mini/côté pour faire confiance au fit (> window_min_pix).
  double max_width_ratio_variation = 1.5;    ///< Ratio largeur max/min toléré entre bas/milieu/haut de la BEV.
  };
