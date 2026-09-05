#pragma once

/// @file
/// @brief Polynôme de voie d'ordre 2 paramétré par y : x = a*y^2 + b*y + c.

#include <opencv2/core.hpp>

#include <vector>

/// @brief Nombre minimal de points par défaut pour un fit fiable.
static const int LANE_POLYNOMIAL_DEFAULT_MIN_POINTS = 3;

/// @brief Polynôme de voie d'ordre 2. Paramétré par y (voies ~verticales) pour
/// éviter les pentes infinies. `valid` distingue un fit réussi d'un côté absent
/// (aléa de la route → drapeau, pas d'assert).
struct LanePolynomial
  {
  double quadratic_coefficient = 0.0;  ///< Coefficient du terme y^2.
  double linear_coefficient = 0.0;     ///< Coefficient du terme y.
  double constant_coefficient = 0.0;   ///< Terme constant.
  bool valid = false;                  ///< true si le fit a réussi.
  int point_count = 0;                 ///< Nombre de points reçus en entrée du fit (0 si aucun).

  /// @brief Évalue x pour un y donné.
  /// @param p_y Ordonnée (pixels BEV).
  /// @return Abscisse x correspondante.
  double eval_at( double p_y ) const;

  /// @brief Ajuste un polynôme d'ordre 2 aux points (moindres carrés).
  /// @param p_points     Points (x, y) à ajuster.
  /// @param p_min_points Nombre minimal de points requis.
  /// @return Polynôme ; valid=false si trop peu de points.
  static LanePolynomial fit( const ::std::vector< ::cv::Point >& p_points,
                             int p_min_points = LANE_POLYNOMIAL_DEFAULT_MIN_POINTS );
  };
