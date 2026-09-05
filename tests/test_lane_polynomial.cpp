#include "doctest.h"

#include <cmath>
#include <vector>

#include "LanePolynomial/LanePolynomial.h"

TEST_CASE( "fit retrouve une parabole connue x = 2y^2 + 3y + 5" )
{
  ::std::vector< ::cv::Point > points;

  for ( int y = 0; y <= 100; y += 5 )
    {
    const int x = static_cast< int >( ::std::lround( ( 2.0 * y * y ) + ( 3.0 * y ) + 5.0 ) );
    points.emplace_back( x, y );
    }

  const LanePolynomial poly = LanePolynomial::fit( points );

  REQUIRE( poly.valid );
  CHECK( poly.quadratic_coefficient == doctest::Approx( 2.0 ).epsilon( 0.01 ) );
  CHECK( poly.linear_coefficient == doctest::Approx( 3.0 ).epsilon( 0.05 ) );
  CHECK( poly.constant_coefficient == doctest::Approx( 5.0 ).epsilon( 1.0 ) );
  CHECK( poly.point_count == static_cast< int >( points.size() ) );
}

TEST_CASE( "fit renvoie invalid si trop peu de points" )
{
  const ::std::vector< ::cv::Point > points = { { 10, 0 }, { 12, 5 } };

  const LanePolynomial poly = LanePolynomial::fit( points );

  CHECK_FALSE( poly.valid );
}

TEST_CASE( "fit renseigne point_count meme quand invalid" )
{
  const ::std::vector< ::cv::Point > points = { { 10, 0 }, { 12, 5 } };

  const LanePolynomial poly = LanePolynomial::fit( points );

  CHECK_FALSE( poly.valid );
  CHECK( poly.point_count == 2 );
}

TEST_CASE( "eval_at evalue le polynome" )
{
  LanePolynomial poly;
  poly.quadratic_coefficient = 1;
  poly.linear_coefficient = 0;
  poly.constant_coefficient = 0;
  poly.valid = true;

  CHECK( poly.eval_at( 3.0 ) == doctest::Approx( 9.0 ) );
}
