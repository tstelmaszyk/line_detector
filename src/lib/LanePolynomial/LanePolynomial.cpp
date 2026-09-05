/// @file
/// @brief Implémentation de LanePolynomial (fit moindres carrés, évaluation).

#include <opencv2/core.hpp>

#include <cmath>
#include <vector>

#include "LanePolynomial/LanePolynomial.h"
#include "SmartAssert/SmartAssert.h"

namespace
{

const int POLYNOMIAL_COEFFICIENT_COUNT = 3;  ///< Nombre de coefficients (ordre 2).
const int QUADRATIC_INDEX = 0;               ///< Indice du coefficient y^2.
const int LINEAR_INDEX = 1;                  ///< Indice du coefficient y.
const int CONSTANT_INDEX = 2;                ///< Indice du terme constant.

} // namespace

double LanePolynomial::eval_at( double p_y ) const
{
  const double x = ( quadratic_coefficient * p_y * p_y )
                 + ( linear_coefficient * p_y )
                 + constant_coefficient;

  SMART_ASSERT( ::std::isfinite( x ), "LanePolynomial::eval_at a produit une valeur non finie" );

  return x;
}

LanePolynomial LanePolynomial::fit( const ::std::vector< ::cv::Point >& p_points, int p_min_points )
{
  LanePolynomial poly;

  const int point_count = static_cast< int >( p_points.size() );
  poly.point_count = point_count;

  if ( point_count < p_min_points )
    {
    poly.valid = false;
    return poly;
    }

  ::cv::Mat vandermonde( point_count, POLYNOMIAL_COEFFICIENT_COUNT, CV_64F );
  ::cv::Mat observed_x( point_count, 1, CV_64F );

  for ( int index = 0; index < point_count; ++index )
    {
    const double y = static_cast< double >( p_points[index].y );

    vandermonde.at< double >( index, QUADRATIC_INDEX ) = y * y;
    vandermonde.at< double >( index, LINEAR_INDEX ) = y;
    vandermonde.at< double >( index, CONSTANT_INDEX ) = 1.0;
    observed_x.at< double >( index, 0 ) = static_cast< double >( p_points[index].x );
    }

  ::cv::Mat coefficients;
  const bool solved = ::cv::solve( vandermonde, observed_x, coefficients, ::cv::DECOMP_SVD );

  SMART_ASSERT( solved, "cv::solve (SVD) a echoue sur un systeme non vide" );

  poly.quadratic_coefficient = coefficients.at< double >( QUADRATIC_INDEX, 0 );
  poly.linear_coefficient = coefficients.at< double >( LINEAR_INDEX, 0 );
  poly.constant_coefficient = coefficients.at< double >( CONSTANT_INDEX, 0 );
  poly.valid = true;

  return poly;
}
