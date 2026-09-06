# LaneTracker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add temporal smoothing of the `LaneModel` across video frames (EMA on the lane polynomial coefficients, with short-loss coasting), toggleable via `LaneConfig`, without breaking the documented "the detection library stays stateless, callable frame by frame" invariant.

**Architecture:** A new stateful `LaneTracker` component lives in `line_detector_lib` (like `LaneGeometry`/`LaneQuality`) but is **not** owned by `DetectLines` — it is owned by `PipelineRunner` (the only place that already iterates over frames), which calls `m_tracker.update(raw_model)` right after `m_detector.compute(p_frame)` and before `render()`/observer notification. `DetectLines` itself is untouched.

**Tech Stack:** C++17, OpenCV (`cv::Mat`, `cv::polylines`, `cv::putText`), doctest for unit tests, CMake.

**Spec:** `docs/superpowers/specs/2026-09-06-lane-tracker-design.md`

## Global Constraints

- `line_detector_lib` must remain free of `argv`/`stdout`/capture-loop code (existing rule, unaffected by this plan).
- `DetectLines::compute()` stays `const` and stateless — `PipelineRunner` keeps holding it as `const DetectLines&`.
- New `LaneConfig` fields: `bool lane_tracker_enabled = true;`, `double lane_tracker_alpha = 0.3;`, `int lane_tracker_max_coast_frames = 10;` — exact names and defaults, used verbatim across every task below.
- New `LaneModel` field: `bool coasted = false;`.
- CSV column order: `frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms` — `coasted` inserted right after `reconstructed`, before `compute_ms` (append-only, does not shift any existing column).
- All new/modified C++ code follows the file's existing style: `#pragma once`, `/// @file`/`/// @brief` header comment, `::` qualification on every std/cv symbol, `p_`/`m_` prefixes, Allman braces indented one level past the brace they belong to (as in every file read during planning).
- Every existing automated test must still pass after every task; no test is deleted, only extended or adapted to the new constructor signature.

---

### Task 1: `LaneConfig` / `LaneModel` — new fields

**Files:**
- Modify: `src/lib/LaneConfig/LaneConfig.h`
- Modify: `src/lib/LaneModel/LaneModel.h`
- Modify: `tests/test_lane_config.cpp`

**Interfaces:**
- Produces: `LaneConfig::lane_tracker_enabled` (`bool`, default `true`), `LaneConfig::lane_tracker_alpha` (`double`, default `0.3`), `LaneConfig::lane_tracker_max_coast_frames` (`int`, default `10`), `LaneModel::coasted` (`bool`, default `false`). Every later task reads these exact names.

- [ ] **Step 1: Write the failing test**

Append to the existing `TEST_CASE` in `tests/test_lane_config.cpp` (do not create a new `TEST_CASE`, extend the current one):

```cpp
TEST_CASE( "LaneConfig fournit des valeurs par defaut saines" )
{
  LaneConfig config;

  CHECK( config.window_count > 0 );
  CHECK( config.window_margin > 0 );
  CHECK( config.window_min_pix > 0 );
  CHECK( config.white_threshold > 0 );
  CHECK( config.src_top_width_ratio > 0.0f );
  CHECK( config.src_top_y_ratio > 0.0f );
  CHECK( config.src_top_y_ratio < 1.0f );
  CHECK( config.src_bottom_width_ratio > 0.0f );
  CHECK( config.src_bottom_width_ratio <= 0.5f );
  CHECK( config.src_bottom_y_ratio > 0.0f );
  CHECK( config.src_bottom_y_ratio <= 1.0f );
  CHECK( config.default_lane_width_px == doctest::Approx( 0.0 ) );

  CHECK( config.min_quality_points > 0 );
  CHECK( config.min_quality_points > config.window_min_pix );
  CHECK( config.max_width_ratio_variation > 1.0 );

  CHECK( config.lane_tracker_enabled );
  CHECK( config.lane_tracker_alpha > 0.0 );
  CHECK( config.lane_tracker_alpha <= 1.0 );
  CHECK( config.lane_tracker_max_coast_frames > 0 );
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j'
```
Expected: build FAILS with "no member named 'lane_tracker_enabled' in 'LaneConfig'" (and similarly for the other two fields).

- [ ] **Step 3: Add the fields**

In `src/lib/LaneConfig/LaneConfig.h`, after the `// --- Qualité / confiance (LaneQuality) ---` block (the last block in the struct), add:

```cpp
  // --- Lissage temporel (LaneTracker) ---
  bool lane_tracker_enabled = true;          ///< false = LaneTracker::update() est un pass-through.
  double lane_tracker_alpha = 0.3;           ///< Poids de la mesure fraiche dans l'EMA (0 < alpha <= 1).
  int lane_tracker_max_coast_frames = 10;    ///< Frames de coasting tolerees avant reset (~0.3s a 30 fps).
```

In `src/lib/LaneModel/LaneModel.h`, after the `reconstructed` field, add:

```cpp
  bool coasted = false;              ///< true si ce frame n'a pas de detection fraiche (LaneTracker reconduit le dernier modele lisse).
```

- [ ] **Step 4: Run test to verify it passes**

Run the same command as Step 2.
Expected: `line_detector_tests` builds; running `/tmp/build/line_detector_tests` shows all cases passing, including "LaneConfig fournit des valeurs par defaut saines".

- [ ] **Step 5: Commit**

```bash
git add src/lib/LaneConfig/LaneConfig.h src/lib/LaneModel/LaneModel.h tests/test_lane_config.cpp
git commit -m "feat(lib): ajoute les reglages LaneConfig et le champ LaneModel::coasted pour le lissage temporel"
```

---

### Task 2: `LaneTracker` — composant de lissage EMA avec coasting

**Files:**
- Create: `src/lib/LaneTracker/LaneTracker.h`
- Create: `src/lib/LaneTracker/LaneTracker.cpp`
- Create: `tests/test_lane_tracker.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `LaneConfig::lane_tracker_enabled/alpha/max_coast_frames` (Task 1), `LaneModel::coasted` (Task 1), `LanePolynomial` (`quadratic_coefficient`/`linear_coefficient`/`constant_coefficient`/`valid`/`point_count`, `eval_at(double)`), `LaneGeometry::compute(LaneModel, const VideoCaracteristics&, const LaneConfig&)` (static, existing).
- Produces: `class LaneTracker` with constructor `LaneTracker(const VideoCaracteristics&, const LaneConfig&, ImageSink&)`, method `LaneModel update(const LaneModel& p_raw_model)`, method `void reset()`. Task 3 constructs and calls this exact interface.

- [ ] **Step 1: Write the failing test file**

Create `tests/test_lane_tracker.cpp`:

```cpp
#include "doctest.h"

#include <opencv2/core.hpp>

#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneGeometry/LaneGeometry.h"
#include "LaneModel/LaneModel.h"
#include "LaneTracker/LaneTracker.h"
#include "VideoCaracteristics/VideoCaracteristics.h"

namespace
{

/// @brief Polynome constant (voie verticale), comme dans test_lane_geometry.cpp.
LanePolynomial straight( double p_x )
{
  LanePolynomial poly;
  poly.quadratic_coefficient = 0.0;
  poly.linear_coefficient = 0.0;
  poly.constant_coefficient = p_x;
  poly.valid = true;
  poly.point_count = 500;
  return poly;
}

/// @brief Construit un LaneModel brut "detecte", comme le ferait DetectLines::compute
/// (fit + LaneGeometry::compute), pour nourrir LaneTracker::update avec une entree realiste.
LaneModel make_detected_model( double p_left_x,
                               double p_right_x,
                               const VideoCaracteristics& p_video,
                               const LaneConfig& p_config )
{
  LaneModel model;
  model.left = straight( p_left_x );
  model.right = straight( p_right_x );
  return LaneGeometry::compute( model, p_video, p_config );
}

/// @brief Construit un LaneModel brut "non detecte" (aucun cote valide).
LaneModel make_missed_model( const VideoCaracteristics& p_video, const LaneConfig& p_config )
{
  LaneModel model;
  return LaneGeometry::compute( model, p_video, p_config );
}

} // namespace

TEST_CASE( "LaneTracker desactive : pass-through strict, aucun etat accumule" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_enabled = false;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  const LaneModel first_raw = make_detected_model( 440.0, 840.0, video, config );
  const LaneModel first_result = tracker.update( first_raw );

  CHECK( first_result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( first_result.right.constant_coefficient == doctest::Approx( 840.0 ) );
  CHECK_FALSE( first_result.coasted );

  const LaneModel second_raw = make_detected_model( 460.0, 860.0, video, config );
  const LaneModel second_result = tracker.update( second_raw );

  // Pas d'etat accumule : le second resultat est exactement le second brut,
  // pas un melange avec le premier appel.
  CHECK( second_result.left.constant_coefficient == doctest::Approx( 460.0 ) );
  CHECK( second_result.right.constant_coefficient == doctest::Approx( 860.0 ) );
}

TEST_CASE( "LaneTracker : premiere detection -> seed identite" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  const LaneModel raw = make_detected_model( 440.0, 840.0, video, config );
  const LaneModel result = tracker.update( raw );

  REQUIRE( result.lane_detected );
  CHECK( result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 840.0 ) );
  CHECK_FALSE( result.coasted );
}

TEST_CASE( "LaneTracker : deuxieme detection -> EMA vers la nouvelle mesure" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_alpha = 0.3;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );  // seed
  const LaneModel result = tracker.update( make_detected_model( 540.0, 940.0, video, config ) );

  // smoothed = alpha*raw + (1-alpha)*previous = 0.3*540 + 0.7*440 = 470 (idem +30 a droite).
  CHECK( result.left.constant_coefficient == doctest::Approx( 470.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 870.0 ) );
  CHECK_FALSE( result.coasted );
}

TEST_CASE( "LaneTracker : perte courte -> coasting sur le dernier modele lisse" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_max_coast_frames = 3;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );
  const LaneModel result = tracker.update( make_missed_model( video, config ) );

  REQUIRE( result.lane_detected );
  CHECK( result.coasted );
  CHECK( result.left.constant_coefficient == doctest::Approx( 440.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 840.0 ) );
}

TEST_CASE( "LaneTracker : perte prolongee -> reset puis reseed identite" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  config.lane_tracker_max_coast_frames = 2;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );       // seed
  tracker.update( make_missed_model( video, config ) );                       // miss 1/2 : coasting
  tracker.update( make_missed_model( video, config ) );                       // miss 2/2 : coasting
  const LaneModel after_reset = tracker.update( make_missed_model( video, config ) );  // miss 3 : reset

  CHECK_FALSE( after_reset.lane_detected );
  CHECK_FALSE( after_reset.coasted );

  // Le prochain fit valide reseed a l'identite, pas de melange avec l'etat perime.
  const LaneModel reseeded = tracker.update( make_detected_model( 900.0, 1300.0, video, config ) );
  CHECK( reseeded.left.constant_coefficient == doctest::Approx( 900.0 ) );
  CHECK( reseeded.right.constant_coefficient == doctest::Approx( 1300.0 ) );
}

TEST_CASE( "LaneTracker : reset() manuel efface l'etat" )
{
  ::cv::Mat ref( 720, 1280, CV_8UC3 );
  VideoCaracteristics video( ref );
  LaneConfig config;
  NullImageSink sink;
  LaneTracker tracker( video, config, sink );

  tracker.update( make_detected_model( 440.0, 840.0, video, config ) );
  tracker.reset();
  const LaneModel result = tracker.update( make_detected_model( 900.0, 1300.0, video, config ) );

  CHECK( result.left.constant_coefficient == doctest::Approx( 900.0 ) );
  CHECK( result.right.constant_coefficient == doctest::Approx( 1300.0 ) );
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j'
```
Expected: build FAILS — `tests/test_lane_tracker.cpp` is picked up by the `file(GLOB TEST_SOURCES tests/test_*.cpp)` automatically, but fails with "LaneTracker/LaneTracker.h: No such file or directory".

- [ ] **Step 3: Create `src/lib/LaneTracker/LaneTracker.h`**

```cpp
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

    LanePolynomial m_smoothed_left;   ///< Etat lisse, cote gauche.
    LanePolynomial m_smoothed_right;  ///< Etat lisse, cote droit.
    bool m_has_state;                 ///< true des qu'un premier fit valide a ete vu.
    int m_miss_streak;                ///< Detections manquees consecutives.
  };
```

- [ ] **Step 4: Create `src/lib/LaneTracker/LaneTracker.cpp`**

```cpp
/// @file
/// @brief Implementation de LaneTracker.

#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <vector>

#include "LaneGeometry/LaneGeometry.h"
#include "LaneTracker/LaneTracker.h"

namespace
{

const int TEXT_BUFFER_SIZE = 96;                        ///< Taille du tampon texte de debug.
const int FIT_LINE_THICKNESS = 2;                       ///< Epaisseur des polynomes traces (debug).
const double TRACKER_FONT_SCALE = 0.8;                  ///< Echelle de la police du texte de debug.
const int TRACKER_TEXT_THICKNESS = 2;                   ///< Epaisseur du texte de debug.
const ::cv::Point TRACKER_TEXT_ORIGIN( 20, 40 );        ///< Origine du texte de debug.
const ::cv::Scalar COLOR_RAW_LEFT( 0, 0, 255 );         ///< Rouge : fit brut gauche (debug).
const ::cv::Scalar COLOR_RAW_RIGHT( 255, 0, 0 );        ///< Bleu : fit brut droit (debug).
const ::cv::Scalar COLOR_SMOOTHED_LEFT( 0, 255, 0 );    ///< Vert : fit lisse gauche (debug).
const ::cv::Scalar COLOR_SMOOTHED_RIGHT( 0, 255, 255 ); ///< Jaune : fit lisse droit (debug).
const ::cv::Scalar COLOR_TRACKER_TEXT( 0, 0, 255 );     ///< Rouge : texte de debug.

/// @brief Melange un coefficient brut avec sa valeur lissee precedente (EMA).
double blend( double p_previous, double p_raw, double p_alpha )
  {
  return ( p_alpha * p_raw ) + ( ( 1.0 - p_alpha ) * p_previous );
  }

/// @brief Lisse un polynome par EMA, coefficient par coefficient. Conserve
/// valid/point_count du brut (deja true/renseigne pour un cote detecte).
LanePolynomial blend_polynomial( const LanePolynomial& p_previous,
                                 const LanePolynomial& p_raw,
                                 double p_alpha )
  {
  LanePolynomial result = p_raw;
  result.quadratic_coefficient = blend( p_previous.quadratic_coefficient, p_raw.quadratic_coefficient, p_alpha );
  result.linear_coefficient = blend( p_previous.linear_coefficient, p_raw.linear_coefficient, p_alpha );
  result.constant_coefficient = blend( p_previous.constant_coefficient, p_raw.constant_coefficient, p_alpha );
  return result;
  }

} // namespace

LaneTracker::LaneTracker( const VideoCaracteristics& p_video,
                          const LaneConfig& p_config,
                          ImageSink& p_debug_sink )
  : m_video_properties( p_video ),
    m_config( p_config ),
    m_debug_sink( p_debug_sink ),
    m_smoothed_left(),
    m_smoothed_right(),
    m_has_state( false ),
    m_miss_streak( 0 )
{
}

void LaneTracker::reset()
{
  m_smoothed_left = LanePolynomial();
  m_smoothed_right = LanePolynomial();
  m_has_state = false;
  m_miss_streak = 0;
}

LaneModel LaneTracker::update( const LaneModel& p_raw_model )
{
  if ( !m_config.lane_tracker_enabled )
    {
    return p_raw_model;
    }

  if ( p_raw_model.lane_detected )
    {
    m_miss_streak = 0;

    if ( !m_has_state )
      {
      m_smoothed_left = p_raw_model.left;
      m_smoothed_right = p_raw_model.right;
      m_has_state = true;
      }
    else
      {
      m_smoothed_left = blend_polynomial( m_smoothed_left, p_raw_model.left, m_config.lane_tracker_alpha );
      m_smoothed_right = blend_polynomial( m_smoothed_right, p_raw_model.right, m_config.lane_tracker_alpha );
      }

    LaneModel smoothed_model;
    smoothed_model.left = m_smoothed_left;
    smoothed_model.right = m_smoothed_right;
    smoothed_model.reconstructed = p_raw_model.reconstructed;
    smoothed_model.coasted = false;
    smoothed_model = LaneGeometry::compute( smoothed_model, m_video_properties, m_config );

    if ( m_debug_sink.is_enabled() )
      {
      draw_debug_trace( p_raw_model, false );
      }

    return smoothed_model;
    }

  ++m_miss_streak;

  const bool can_coast = ( m_has_state && ( m_miss_streak <= m_config.lane_tracker_max_coast_frames ) );

  if ( can_coast )
    {
    LaneModel coasted_model;
    coasted_model.left = m_smoothed_left;
    coasted_model.right = m_smoothed_right;
    coasted_model.reconstructed = false;
    coasted_model.coasted = true;
    coasted_model = LaneGeometry::compute( coasted_model, m_video_properties, m_config );

    if ( m_debug_sink.is_enabled() )
      {
      draw_debug_trace( p_raw_model, true );
      }

    return coasted_model;
    }

  m_has_state = false;

  if ( m_debug_sink.is_enabled() )
    {
    draw_debug_trace( p_raw_model, false );
    }

  return p_raw_model;
}

void LaneTracker::draw_debug_trace( const LaneModel& p_raw_model, bool p_coasted ) const
{
  ::cv::Mat canvas( m_video_properties.image_size, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );
  const int height = static_cast< int >( m_video_properties.height_pixel );

  ::std::vector< ::cv::Point > raw_left_points;
  ::std::vector< ::cv::Point > raw_right_points;
  ::std::vector< ::cv::Point > smoothed_left_points;
  ::std::vector< ::cv::Point > smoothed_right_points;

  for ( int y = 0; y < height; ++y )
    {
    if ( p_raw_model.left.valid )
      {
      raw_left_points.emplace_back( ::cvRound( p_raw_model.left.eval_at( y ) ), y );
      }

    if ( p_raw_model.right.valid )
      {
      raw_right_points.emplace_back( ::cvRound( p_raw_model.right.eval_at( y ) ), y );
      }

    if ( m_has_state )
      {
      smoothed_left_points.emplace_back( ::cvRound( m_smoothed_left.eval_at( y ) ), y );
      smoothed_right_points.emplace_back( ::cvRound( m_smoothed_right.eval_at( y ) ), y );
      }
    }

  if ( !raw_left_points.empty() )
    {
    ::cv::polylines( canvas, raw_left_points, false, COLOR_RAW_LEFT, FIT_LINE_THICKNESS );
    }

  if ( !raw_right_points.empty() )
    {
    ::cv::polylines( canvas, raw_right_points, false, COLOR_RAW_RIGHT, FIT_LINE_THICKNESS );
    }

  if ( !smoothed_left_points.empty() )
    {
    ::cv::polylines( canvas, smoothed_left_points, false, COLOR_SMOOTHED_LEFT, FIT_LINE_THICKNESS );
    }

  if ( !smoothed_right_points.empty() )
    {
    ::cv::polylines( canvas, smoothed_right_points, false, COLOR_SMOOTHED_RIGHT, FIT_LINE_THICKNESS );
    }

  char text[TEXT_BUFFER_SIZE];
  ::std::snprintf( text,
                   sizeof( text ),
                   "alpha=%.2f  miss_streak=%d  coasted=%d",
                   m_config.lane_tracker_alpha,
                   m_miss_streak,
                   p_coasted ? 1 : 0 );

  ::cv::putText( canvas,
                text,
                TRACKER_TEXT_ORIGIN,
                ::cv::FONT_HERSHEY_SIMPLEX,
                TRACKER_FONT_SCALE,
                COLOR_TRACKER_TEXT,
                TRACKER_TEXT_THICKNESS );

  m_debug_sink.save( "debug_04c_tracker.jpg", canvas );
}
```

- [ ] **Step 5: Register the new source file in CMake**

In `CMakeLists.txt`, in the `LANE_SOURCES` list, add a line at the end (before the closing `)`):

```cmake
set(LANE_SOURCES
    src/lib/DetectLines/DetectLines.cpp
    src/lib/ImageSink/DiskImageSink.cpp
    src/lib/LanePolynomial/LanePolynomial.cpp
    src/lib/LaneGeometry/LaneGeometry.cpp
    src/lib/LaneMask/LaneMask.cpp
    src/lib/LaneQuality/LaneQuality.cpp
    src/lib/PerspectiveView/PerspectiveView.cpp
    src/lib/SlidingWindowSearch/SlidingWindowSearch.cpp
    src/lib/LaneOverlay/LaneOverlay.cpp
    src/lib/LaneTracker/LaneTracker.cpp
)
```

- [ ] **Step 6: Run tests to verify they pass**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```
Expected: PASS — all `test_lane_tracker.cpp` cases green, and every pre-existing test still green (nothing else was touched).

- [ ] **Step 7: Commit**

```bash
git add src/lib/LaneTracker/LaneTracker.h src/lib/LaneTracker/LaneTracker.cpp tests/test_lane_tracker.cpp CMakeLists.txt
git commit -m "feat(lib): ajoute LaneTracker (lissage EMA des polynomes + coasting sur perte courte)"
```

---

### Task 3: Intégration dans `PipelineRunner`

**Files:**
- Modify: `src/app/PipelineRunner/PipelineRunner.h`
- Modify: `src/app/PipelineRunner/PipelineRunner.cpp`
- Modify: `tests/test_pipeline_runner.cpp`
- Modify: `tests/test_video_integration.cpp`

**Interfaces:**
- Consumes: `LaneTracker` (Task 2) — constructor `LaneTracker(const VideoCaracteristics&, const LaneConfig&, ImageSink&)`, method `LaneModel update(const LaneModel&)`.
- Produces: `PipelineRunner` constructor now takes a `LaneTracker& p_tracker` (new required 3rd parameter, right after `p_detector`). Every `PipelineRunner` construction site elsewhere (Task 5's `main.cpp`) must pass one.

- [ ] **Step 1: Write the failing tests — update existing call sites**

In `tests/test_pipeline_runner.cpp`, add the include:

```cpp
#include "LaneTracker/LaneTracker.h"
```
(alphabetically, between `"ImageSink/NullImageSink.h"` and `"LaneConfig/LaneConfig.h"`).

Then replace **every** occurrence (there are 6, one per `TEST_CASE`, all textually identical) of:

```cpp
  PipelineRunner runner( source, detector, observers, stop_requested );
```

with:

```cpp
  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
```

(Every `TEST_CASE` in this file already declares `video`, `config` and `debug_sink` with those exact names right before this line, so the replacement is mechanical and identical in all 6 places.)

Then append a **new** `TEST_CASE` at the end of the file (after the last existing one), proving the wiring actually calls the tracker, not just that it compiles:

```cpp
TEST_CASE( "PipelineRunner : appelle le LaneTracker (coasting sur une frame sans detection)" )
{
  const ::cv::Mat first_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
  VideoCaracteristics video( first_frame );
  LaneConfig config;
  config.default_lane_width_px = RUNNER_TEST_WIDTH * RUNNER_LANE_WIDTH_RATIO;
  config.lane_tracker_max_coast_frames = 5;
  NullImageSink debug_sink;
  const DetectLines detector( video, config, debug_sink );
  LaneTracker tracker( video, config, debug_sink );

  // Frame 0 : sans marquage (lane_detected=false). Frame 1 : avec marquages.
  class MixedDetectionFrameSource : public FrameSource
    {
    public:
      MixedDetectionFrameSource() : m_step( 0 ) { }

      bool read( ::cv::Mat& p_frame ) override
        {
        if ( 0 == m_step )
          {
          const ::cv::Scalar background( RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY, RUNNER_BACKGROUND_GRAY );
          p_frame = ::cv::Mat( RUNNER_TEST_HEIGHT, RUNNER_TEST_WIDTH, CV_8UC3, background );
          ++m_step;
          return true;
          }

        if ( 1 == m_step )
          {
          p_frame = make_lane_frame( RUNNER_LEFT_LINE_X, RUNNER_RIGHT_LINE_X );
          ++m_step;
          return true;
          }

        return false;
        }

    private:
      int m_step;
    };

  class ModelRecorder : public FrameObserver
    {
    public:
      ModelRecorder() : m_models() { }

      void on_frame( int p_frame_index,
                     const LaneModel& p_model,
                     const ::cv::Mat& p_annotated_frame,
                     double p_compute_ms,
                     double p_render_ms ) override
        {
        (void) p_frame_index;
        (void) p_annotated_frame;
        (void) p_compute_ms;
        (void) p_render_ms;
        m_models.push_back( p_model );
        }

      bool needs_annotated_frame() const override { return false; }

      const ::std::vector< LaneModel >& models() const { return m_models; }

    private:
      ::std::vector< LaneModel > m_models;
    };

  MixedDetectionFrameSource source;
  ModelRecorder recorder;
  ::std::vector< FrameObserver* > observers;
  observers.push_back( &recorder );
  const ::std::atomic< bool > stop_requested( false );

  // Premiere frame (passee directement a run) : avec marquages -> detection fraiche.
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
  RunStats stats;
  const int status = runner.run( first_frame, stats );

  CHECK( EXIT_SUCCESS == status );
  REQUIRE( 3 == static_cast< int >( recorder.models().size() ) );
  CHECK( recorder.models()[0].lane_detected );
  CHECK_FALSE( recorder.models()[0].coasted );
  CHECK( recorder.models()[1].lane_detected );   // frame sans marquage -> coasting
  CHECK( recorder.models()[1].coasted );
  CHECK( recorder.models()[2].lane_detected );   // detection fraiche a nouveau
  CHECK_FALSE( recorder.models()[2].coasted );
}
```

In `tests/test_video_integration.cpp`, add the include:

```cpp
#include "LaneTracker/LaneTracker.h"
```
(alphabetically, between `"ImageSink/NullImageSink.h"` and `"LaneConfig/LaneConfig.h"`).

Then, in the single `TEST_CASE`, right after the existing line `config.max_width_ratio_variation = 10.0;`, add a new line with an explanatory comment (matching the file's existing comment style):

```cpp
  // Tracker desactive : ce test verifie la derive du signal BRUT a travers
  // SlidingWindowSearch/LaneGeometry, pas le lissage (couvert par test_lane_tracker.cpp).
  config.lane_tracker_enabled = false;
```

Then replace the single occurrence of:

```cpp
  PipelineRunner runner( source, detector, observers, stop_requested );
```

with:

```cpp
  LaneTracker tracker( video, config, debug_sink );
  PipelineRunner runner( source, detector, tracker, observers, stop_requested );
```

- [ ] **Step 2: Run tests to verify they fail**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j'
```
Expected: build FAILS — `PipelineRunner`'s constructor does not yet accept a `LaneTracker&` (too many arguments).

- [ ] **Step 3: Update `PipelineRunner.h`**

Replace:

```cpp
#include "DetectLines/DetectLines.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameSource/FrameSource.h"
#include "RunStats/RunStats.h"
#include "projectTypes.h"
```

with:

```cpp
#include "DetectLines/DetectLines.h"
#include "FrameObserver/FrameObserver.h"
#include "FrameSource/FrameSource.h"
#include "LaneTracker/LaneTracker.h"
#include "RunStats/RunStats.h"
#include "projectTypes.h"
```

Replace:

```cpp
    /// @param p_frame_source    Source de frames.
    /// @param p_detector        Pipeline de détection.
    /// @param p_observers       Observateurs notifiés à chaque frame.
    /// @param p_stop_requested  Drapeau d'arrêt (levé par le handler SIGINT).
    PipelineRunner( FrameSource& p_frame_source,
                    const DetectLines& p_detector,
                    const ::std::vector< FrameObserver* >& p_observers,
                    const ::std::atomic< bool >& p_stop_requested );
```

with:

```cpp
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
```

Replace:

```cpp
    FrameSource& m_frame_source;                      ///< Source de frames.
    const DetectLines& m_detector;                    ///< Pipeline de détection.
    ::std::vector< FrameObserver* > m_observers;      ///< Observateurs notifiés.
```

with:

```cpp
    FrameSource& m_frame_source;                      ///< Source de frames.
    const DetectLines& m_detector;                    ///< Pipeline de détection.
    LaneTracker& m_tracker;                            ///< Lissage temporel du LaneModel.
    ::std::vector< FrameObserver* > m_observers;      ///< Observateurs notifiés.
```

Also update the class docstring line that currently reads "la bibliothèque de détection reste sans état, appelable frame par frame" — it is still true (`DetectLines` itself is unchanged); leave it as is, it does not need editing.

- [ ] **Step 4: Update `PipelineRunner.cpp`**

Replace the constructor:

```cpp
PipelineRunner::PipelineRunner( FrameSource& p_frame_source,
                                const DetectLines& p_detector,
                                const ::std::vector< FrameObserver* >& p_observers,
                                const ::std::atomic< bool >& p_stop_requested )
  : m_frame_source( p_frame_source ),
    m_detector( p_detector ),
    m_observers( p_observers ),
    m_stop_requested( p_stop_requested ),
    m_render_needed( false )
```

with:

```cpp
PipelineRunner::PipelineRunner( FrameSource& p_frame_source,
                                const DetectLines& p_detector,
                                LaneTracker& p_tracker,
                                const ::std::vector< FrameObserver* >& p_observers,
                                const ::std::atomic< bool >& p_stop_requested )
  : m_frame_source( p_frame_source ),
    m_detector( p_detector ),
    m_tracker( p_tracker ),
    m_observers( p_observers ),
    m_stop_requested( p_stop_requested ),
    m_render_needed( false )
```

Replace, in `process_frame`:

```cpp
  const ::std::chrono::steady_clock::time_point compute_start = ::std::chrono::steady_clock::now();

  const LaneModel model = m_detector.compute( p_frame );

  const DurationMs compute_ms = elapsed_ms_since( compute_start );
```

with:

```cpp
  const ::std::chrono::steady_clock::time_point compute_start = ::std::chrono::steady_clock::now();

  const LaneModel raw_model = m_detector.compute( p_frame );
  const LaneModel model = m_tracker.update( raw_model );

  const DurationMs compute_ms = elapsed_ms_since( compute_start );
```

(everything below this point already refers to `model` and needs no further change).

- [ ] **Step 5: Run tests to verify they pass**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```
Expected: PASS — all cases in `test_pipeline_runner.cpp` (including the new coasting test) and `test_video_integration.cpp` green, plus every other test still green.

- [ ] **Step 6: Commit**

```bash
git add src/app/PipelineRunner/PipelineRunner.h src/app/PipelineRunner/PipelineRunner.cpp \
        tests/test_pipeline_runner.cpp tests/test_video_integration.cpp
git commit -m "feat(app): cable LaneTracker dans PipelineRunner entre compute() et render()"
```

---

### Task 4: `LaneModelLogger` — colonne CSV `coasted`

**Files:**
- Modify: `src/app/FrameObserver/LaneModelLogger.cpp`
- Modify: `tests/test_frame_observer.cpp`

**Interfaces:**
- Consumes: `LaneModel::coasted` (Task 1).
- Produces: CSV header/row now include a `coasted` column between `reconstructed` and `compute_ms` — Task 6 documents this exact new format.

- [ ] **Step 1: Write the failing test**

In `tests/test_frame_observer.cpp`, append a new `TEST_CASE` after "LaneModelLogger : grand rayon de courbure -> pas de notation scientifique":

```cpp
TEST_CASE( "LaneModelLogger : colonne coasted, entre reconstructed et compute_ms" )
{
  ::std::ostringstream output;
  LaneModelLogger logger( output );
  const ::cv::Mat frame( OBSERVER_TEST_HEIGHT, OBSERVER_TEST_WIDTH, CV_8UC3, ::cv::Scalar( 0, 0, 0 ) );

  LaneModel model = make_test_model( 0.0 );
  model.coasted = true;
  logger.on_frame( 0, model, frame, TEST_ELAPSED_MS, 0.0 );

  const ::std::string text = output.str();

  CHECK( ::std::string::npos != text.find(
    "frame_index;lane_detected;normalized_offset;lateral_offset_px;"
    "curvature_radius_px;reconstructed;coasted;compute_ms;render_ms" ) );
  // reconstructed=0 (make_test_model), coasted=1 (force ci-dessus), compute_ms=12.500000.
  CHECK( ::std::string::npos != text.find(
    "0;1;0.000000;42.000000;1500.000000;0;1;12.500000;0.000000" ) );
}
```

- [ ] **Step 2: Run test to verify it fails**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```
Expected: build succeeds (the `LaneModel::coasted` field already exists from Task 1) but the new `TEST_CASE` FAILS — the CSV header/row do not yet contain `coasted`.

- [ ] **Step 3: Update `LaneModelLogger.cpp`**

Replace:

```cpp
const ::std::string CSV_HEADER =
  "frame_index;lane_detected;normalized_offset;lateral_offset_px;"
  "curvature_radius_px;reconstructed;compute_ms;render_ms";  ///< En-tête du log.
```

with:

```cpp
const ::std::string CSV_HEADER =
  "frame_index;lane_detected;normalized_offset;lateral_offset_px;"
  "curvature_radius_px;reconstructed;coasted;compute_ms;render_ms";  ///< En-tête du log.
```

Replace:

```cpp
  const int lane_detected_flag = p_model.lane_detected ? 1 : 0;
  const int reconstructed_flag = p_model.reconstructed ? 1 : 0;

  m_output_stream << p_frame_index << FIELD_SEPARATOR
                  << lane_detected_flag << FIELD_SEPARATOR
                  << p_model.normalized_offset << FIELD_SEPARATOR
                  << p_model.lateral_offset_px << FIELD_SEPARATOR
                  << p_model.curvature_radius_px << FIELD_SEPARATOR
                  << reconstructed_flag << FIELD_SEPARATOR
                  << p_compute_ms << FIELD_SEPARATOR
                  << p_render_ms << "\n";
```

with:

```cpp
  const int lane_detected_flag = p_model.lane_detected ? 1 : 0;
  const int reconstructed_flag = p_model.reconstructed ? 1 : 0;
  const int coasted_flag = p_model.coasted ? 1 : 0;

  m_output_stream << p_frame_index << FIELD_SEPARATOR
                  << lane_detected_flag << FIELD_SEPARATOR
                  << p_model.normalized_offset << FIELD_SEPARATOR
                  << p_model.lateral_offset_px << FIELD_SEPARATOR
                  << p_model.curvature_radius_px << FIELD_SEPARATOR
                  << reconstructed_flag << FIELD_SEPARATOR
                  << coasted_flag << FIELD_SEPARATOR
                  << p_compute_ms << FIELD_SEPARATOR
                  << p_render_ms << "\n";
```

- [ ] **Step 4: Run tests to verify they pass**

Same command as Step 2. Expected: PASS, all cases in `test_frame_observer.cpp` green (including the new one) and every other test still green.

- [ ] **Step 5: Commit**

```bash
git add src/app/FrameObserver/LaneModelLogger.cpp tests/test_frame_observer.cpp
git commit -m "feat(app): ajoute la colonne CSV coasted au log LaneModelLogger"
```

---

### Task 5: `main.cpp` — construction et injection du `LaneTracker`

**Files:**
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: `LaneTracker` (Task 2), updated `PipelineRunner` constructor (Task 3).

There is no dedicated unit test for `main.cpp` (it has no logic beyond assembly — `test_main.cpp` is only the doctest entry point). This task is verified by a full build of the `line_detector` executable and one manual run, per the project's existing convention for `main.cpp` changes.

- [ ] **Step 1: Update `src/main.cpp`**

Add the include, alphabetically next to the other lib includes:

```cpp
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "PipelineRunner/PipelineRunner.h"
```
becomes
```cpp
#include "ImageSink/NullImageSink.h"
#include "LaneConfig/LaneConfig.h"
#include "LaneTracker/LaneTracker.h"
#include "PipelineRunner/PipelineRunner.h"
```

Replace:

```cpp
  const DetectLines detector( video_properties, config, *debug_sink );
```

with:

```cpp
  const DetectLines detector( video_properties, config, *debug_sink );
  LaneTracker tracker( video_properties, config, *debug_sink );
```

Replace:

```cpp
  PipelineRunner runner( *frame_source, detector, observers, g_stop_requested );
```

with:

```cpp
  PipelineRunner runner( *frame_source, detector, tracker, observers, g_stop_requested );
```

- [ ] **Step 2: Build the executable**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector -j'
```
Expected: builds with no errors or new warnings (`-Wall -Wextra` is on for `line_detector_app`).

- [ ] **Step 3: Run it manually and check the new debug trace + CSV column**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'mkdir -p /app/out && cd /app && LINE_DETECTOR_DEBUG=1 \
           /tmp/build/line_detector --image img_piste/img2.jpg --record > /tmp/out.csv \
           && head -2 /tmp/out.csv && ls -la out/debug_04c_tracker.jpg'
```
Expected:
- The CSV header line printed by `head -2` contains `coasted` between `reconstructed` and `compute_ms`.
- `out/debug_04c_tracker.jpg` exists (a still image is a single-frame "run", so this is the identity-seed case: raw and smoothed curves should coincide, or the raw ones alone if `img2.jpg` yields no valid fit).
- Exit code `0`, no crash.

- [ ] **Step 4: Run the full test suite one more time (regression check)**

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```
Expected: PASS, all tests green.

- [ ] **Step 5: Commit**

```bash
git add src/main.cpp
git commit -m "feat(app): construit et injecte LaneTracker dans main.cpp"
```

---

### Task 6: Documentation — `CLAUDE.md` et `README.md`

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`

**Interfaces:** none (documentation only).

This task has no automated test; the deliverable is verified by a final read-through checking the doc no longer claims "no temporal filtering exists" and correctly describes the new column/component/file.

- [ ] **Step 1: `CLAUDE.md` — "Présentation" section**

Replace:

```markdown
**Mode vidéo incomplet** : le mode vidéo (fichier + caméra) lit et traite
frame par frame, mais ne fait actuellement **aucun filtrage temporel** entre
les frames (pas de lissage, pas de recherche autour du fit précédent) — le
lissage temporel (`LaneTracker`) et le passage à une sortie métrique restent
au programme, cf. roadmap dans `docs/superpowers/specs/` et section « Suite
prévue ».
```

with:

```markdown
**Mode vidéo** : le mode vidéo (fichier + caméra) lit et traite frame par
frame, et lisse le `LaneModel` à travers les frames via `LaneTracker` (EMA sur
les coefficients des polynômes de voie + coasting sur perte de détection
courte), activable/désactivable par `LaneConfig::lane_tracker_enabled` — cf.
`docs/superpowers/specs/2026-09-06-lane-tracker-design.md`. Restent au
programme : recherche autour du fit précédent, correction de distorsion
caméra, passage à une sortie métrique (cf. section « Suite prévue »).
```

- [ ] **Step 2: `CLAUDE.md` — colonne CSV**

Replace:

```markdown
(`frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;compute_ms;render_ms`,
```

with:

```markdown
(`frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms`,
```

- [ ] **Step 3: `CLAUDE.md` — section Architecture, ajout de `LaneTracker`**

Right after the numbered list of 6 steps (after point 6, `LaneOverlay::render`, and before "Types clés et possession"), add a new paragraph:

```markdown

`LaneTracker` (`LaneTracker.cpp`) lisse le `LaneModel` entre les frames (EMA
sur les coefficients de `LanePolynomial`, coasting sur perte de détection
courte). Ce n'est **pas** une étape de `DetectLines::compute`/`render` : c'est
le seul composant de `line_detector_lib` qui porte un état entre deux appels,
et pour cette raison il n'est pas possédé par `DetectLines` (qui reste sans
état, cf. « Couche application » ci-dessous) mais par `PipelineRunner`, qui
appelle `update()` juste après `compute()` et avant `render()`.
```

- [ ] **Step 4: `CLAUDE.md` — bullet `PipelineRunner` mis à jour**

Replace:

```markdown
- **`PipelineRunner` / `RunStats`** (`PipelineRunner.h/.cpp`) — possède la boucle,
  et seulement elle : lire → `DetectLines::compute` → `DetectLines::render` (si
  et seulement si un observateur le réclame, cf. ci-dessus) → notifier les
  observateurs → compter dans un `RunStats` fourni par l'appelant (`compute_ms`
  et `render_ms` accumulés séparément, non réinitialisé par `run`). S'arrête à
  la fin du flux, sur le drapeau `SIGINT`, ou dès qu'un observateur signale
  `has_fatal_error()` — utile pour qu'un `out/` en échec d'écriture soit détecté
  frame par frame plutôt qu'après coup.
```

with:

```markdown
- **`PipelineRunner` / `RunStats`** (`PipelineRunner.h/.cpp`) — possède la boucle,
  et seulement elle : lire → `DetectLines::compute` → `LaneTracker::update` (lissage
  temporel, cf. Architecture ci-dessus) → `DetectLines::render` (si et seulement si un
  observateur le réclame, cf. ci-dessus) → notifier les observateurs → compter dans un
  `RunStats` fourni par l'appelant (`compute_ms` et `render_ms` accumulés séparément,
  non réinitialisé par `run` ; `compute_ms` inclut `LaneTracker::update`). S'arrête à
  la fin du flux, sur le drapeau `SIGINT`, ou dès qu'un observateur signale
  `has_fatal_error()` — utile pour qu'un `out/` en échec d'écriture soit détecté
  frame par frame plutôt qu'après coup. Possède aussi le `LaneTracker` (référence
  injectée par le constructeur, comme `DetectLines`).
```

- [ ] **Step 5: `CLAUDE.md` — traces de debug**

Replace:

```markdown
- **Traces de debug** : exécuter avec `LINE_DETECTOR_DEBUG` non vide
  (`LINE_DETECTOR_DEBUG=1 ./line_detector`) écrit les étapes intermédiaires :
  `out/debug_01_mask.jpg`, `debug_02_bev.jpg`, `debug_03_windows.jpg` et
  `debug_04_fit.jpg` sont écrits dès que la variable est définie (ils viennent de
  `compute`) ; `debug_05_overlay.jpg` demande **en plus** `--record` (il vient de
  `render`, cf. « Couche application »). Sans la variable, aucune trace
  (`NullImageSink`). Choix **runtime** : pas d'option CMake.
```

with:

```markdown
- **Traces de debug** : exécuter avec `LINE_DETECTOR_DEBUG` non vide
  (`LINE_DETECTOR_DEBUG=1 ./line_detector`) écrit les étapes intermédiaires :
  `out/debug_01_mask.jpg`, `debug_02_bev.jpg`, `debug_03_windows.jpg`,
  `debug_04_fit.jpg` et `debug_04c_tracker.jpg` (polynômes bruts vs lissés par
  `LaneTracker`) sont écrits dès que la variable est définie ; `debug_05_overlay.jpg`
  demande **en plus** `--record` (il vient de `render`, cf. « Couche application »).
  Sans la variable, aucune trace (`NullImageSink`). Choix **runtime** : pas
  d'option CMake.
```

- [ ] **Step 6: `CLAUDE.md` — section "Suite prévue"**

Replace:

```markdown
## Suite prévue

Voir `docs/superpowers/specs/2026-07-10-roadmap-video-lissage-temporel.md`. Le
mode vidéo (fichier + caméra, cf. Couche application ci-dessus) est fait ; reste
au programme : `LaneTracker` (lissage temporel / Kalman), recherche autour du
fit précédent, correction de distorsion caméra, passage métrique.
```

with:

```markdown
## Suite prévue

Voir `docs/superpowers/specs/2026-09-06-lane-tracker-design.md` pour le design
du lissage temporel. Le mode vidéo (fichier + caméra, cf. Couche application
ci-dessus) et le lissage temporel (`LaneTracker`) sont faits ; reste au
programme : recherche autour du fit précédent, correction de distorsion
caméra, passage à une sortie métrique.
```

- [ ] **Step 7: `README.md` — "Ce que fait le programme"**

Replace:

```markdown
Trois sources sont acceptées, mutuellement exclusives : une image fixe
(`--image`), un fichier vidéo (`--video`), ou une caméra en direct
(`--camera`). Le mode vidéo lit et traite frame par frame mais ne fait
aujourd'hui **aucun filtrage temporel** : chaque frame est traitée
indépendamment, sans lissage ni recherche autour du fit précédent. C'est la
prochaine brique prévue (voir [Limites connues](#limites-connues)).
```

with:

```markdown
Trois sources sont acceptées, mutuellement exclusives : une image fixe
(`--image`), un fichier vidéo (`--video`), ou une caméra en direct
(`--camera`). Le mode vidéo lit et traite frame par frame, et lisse le
`LaneModel` à travers les frames (`LaneTracker`, EMA sur les coefficients des
polynômes + coasting sur perte de détection courte). Il n'y a en revanche
toujours pas de recherche localisée autour du fit précédent (voir [Limites
connues](#limites-connues)).
```

- [ ] **Step 8: `README.md` — colonne CSV et son explication**

Replace:

```markdown
```
frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;compute_ms;render_ms
```
```

with:

```markdown
```
frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms
```
```

Replace:

```markdown
Rappel de convention : un `normalized_offset` négatif signifie que le véhicule
est décalé **à gauche**. `reconstructed` vaut `1` quand un seul marquage était
visible et que l'autre a été reconstruit par décalage — le signal reste
exploitable mais dégradé.
```

with:

```markdown
Rappel de convention : un `normalized_offset` négatif signifie que le véhicule
est décalé **à gauche**. `reconstructed` vaut `1` quand un seul marquage était
visible et que l'autre a été reconstruit par décalage — le signal reste
exploitable mais dégradé. `coasted` vaut `1` quand cette frame n'a pas de
détection fraîche et que `LaneTracker` reconduit le dernier modèle lissé
connu — signal figé, à distinguer d'une détection fraîche même si
`lane_detected` reste à `1` dans les deux cas.
```

- [ ] **Step 9: `README.md` — table des traces de debug**

Replace:

```markdown
| `out/debug_04_fit.jpg` | polynômes ajustés tracés sur la vue de dessus | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` ne rejette aucun côté |
| `out/debug_04b_quality.jpg` | fits d'origine + raison du rejet (largeur incohérente ou pixels insuffisants) | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` rejette au moins un côté |
| `out/debug_05_overlay.jpg` | overlay final, identique à `out/output.jpg` | `LINE_DETECTOR_DEBUG` **et** `--record` |
```

with:

```markdown
| `out/debug_04_fit.jpg` | polynômes ajustés tracés sur la vue de dessus | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` ne rejette aucun côté |
| `out/debug_04b_quality.jpg` | fits d'origine + raison du rejet (largeur incohérente ou pixels insuffisants) | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` rejette au moins un côté |
| `out/debug_04c_tracker.jpg` | polynômes bruts vs lissés par `LaneTracker` (rouge/bleu = brut, vert/jaune = lissé) | `LINE_DETECTOR_DEBUG` |
| `out/debug_05_overlay.jpg` | overlay final, identique à `out/output.jpg` | `LINE_DETECTOR_DEBUG` **et** `--record` |
```

- [ ] **Step 10: `README.md` — structure du dépôt**

Replace:

```markdown
| `src/lib/LaneOverlay/` | étape 7 — rendu de l'overlay et du HUD |
| `src/lib/LaneConfig/` | tous les réglages numériques du pipeline |
```

with:

```markdown
| `src/lib/LaneOverlay/` | étape 7 — rendu de l'overlay et du HUD |
| `src/lib/LaneTracker/` | lissage temporel du LaneModel (EMA + coasting), hors des étapes 1-7 : possédé par `PipelineRunner`, pas par `DetectLines` |
| `src/lib/LaneConfig/` | tous les réglages numériques du pipeline |
```

- [ ] **Step 11: `README.md` — "Limites connues"**

Replace:

```markdown
## Limites connues

Le mode vidéo (fichier et caméra) lit et traite chaque frame indépendamment :
il n'y a aujourd'hui aucun filtrage temporel entre les frames : pas de
lissage du signal, pas de recherche localisée autour du fit de la frame
précédente (qui accélérerait le traitement en évitant de relancer une
recherche par histogramme à chaque frame). Restent également au programme la
correction de distorsion caméra (le pipeline suppose une caméra sans
distorsion notable) et le passage d'un signal en pixels à un signal en unités
métriques, qui suppose une calibration caméra/sol supplémentaire.
```

with:

```markdown
## Limites connues

Le mode vidéo (fichier et caméra) lisse désormais le `LaneModel` à travers les
frames (`LaneTracker`), mais il n'y a toujours pas de recherche localisée
autour du fit de la frame précédente (qui accélérerait le traitement en
évitant de relancer une recherche par histogramme à chaque frame). Restent
également au programme la correction de distorsion caméra (le pipeline
suppose une caméra sans distorsion notable) et le passage d'un signal en
pixels à un signal en unités métriques, qui suppose une calibration
caméra/sol supplémentaire.
```

- [ ] **Step 12: Commit**

```bash
git add CLAUDE.md README.md
git commit -m "docs: documente LaneTracker (lissage temporel, colonne CSV coasted, trace debug_04c)"
```

---

## Self-Review Notes

- **Spec coverage:** every section of `2026-09-06-lane-tracker-design.md` maps to a task — architecture/ownership (Task 3), interface/algorithm/config (Tasks 1-2), `LaneModel` field + CSV (Tasks 1, 4), debug trace (Task 2), enable/disable (Tasks 1-2), still-image safety (implied by Task 2's identity-seed tests and Task 5's manual run), tests (Tasks 2-3), doc updates (Task 6). Alternatives/risks sections are informational only, no task needed.
- **Placeholder scan:** no TBD/TODO; every step carries literal code or literal before/after text, not a description of intent.
- **Type consistency:** `LaneTracker::update(const LaneModel&) -> LaneModel` and `reset()` are the only two public methods used anywhere after Task 2, and every later task (3, 5) calls them with these exact names. `LaneConfig`/`LaneModel` field names introduced in Task 1 (`lane_tracker_enabled`, `lane_tracker_alpha`, `lane_tracker_max_coast_frames`, `coasted`) are used identically, spelled the same way, in Tasks 2-6.
