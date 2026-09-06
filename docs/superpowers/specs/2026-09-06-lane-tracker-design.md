# LaneTracker — lissage temporel du LaneModel

Date : 2026-09-06
Statut : validé (brainstorming), en attente de plan d'implémentation

## Contexte

Le mode vidéo (fichier + caméra) traite chaque frame indépendamment : aucun
filtrage temporel entre les frames (cf. CLAUDE.md § Présentation, « Mode vidéo
incomplet »). `DetectLines::compute` est stateless et `const` ; `PipelineRunner`
docstring : « la bibliothèque de détection reste sans état, appelable frame par
frame ». Conséquence en usage réel : le `LaneModel` (polynômes, offset,
courbure) peut sauter d'une frame à l'autre à cause du bruit de la route/de la
caméra, et une frame sans détection exploitable (`lane_detected = false`) coupe
immédiatement le signal de pilotage sans transition.

Cette spec couvre uniquement le **lissage temporel** du signal. Elle ne couvre
pas les autres items de la roadmap (recherche autour du fit précédent,
correction de distorsion caméra, sortie métrique) — chacun mérite son propre
cycle spec → plan → implémentation.

## Objectifs

- Lisser le `LaneModel` à travers les frames pour réduire le bruit
  frame-à-frame, sans casser l'invariant « `line_detector_lib` reste sans état,
  appelable frame par frame » déjà documenté pour `DetectLines`/`PipelineRunner`.
- Tolérer une perte de détection courte (quelques frames) en reconduisant le
  dernier modèle lissé connu, plutôt que de couper le signal immédiatement.
- Rester activable/désactivable par configuration, sans recompilation.
- Ne rien changer pour le mode image fixe (une image reste un cas dégénéré du
  flux, jamais un cas particulier codé en dur).

## Non-objectifs (hors scope de cette spec)

- Recherche autour du fit précédent dans `SlidingWindowSearch` (item roadmap
  séparé).
- Filtre de Kalman (écarté au profit d'une EMA pour cette première itération —
  cf. « Alternatives »).
- Correction de distorsion caméra, sortie métrique.

## Architecture

`LaneTracker` est un nouveau composant de `line_detector_lib`
(`src/lib/LaneTracker/`), au même niveau que `LaneQuality`/`LaneGeometry`, mais
c'est le **seul composant de la lib qui porte un état** entre deux appels.

Pour préserver l'invariant documenté dans `PipelineRunner.h` (« la
bibliothèque de détection reste sans état, appelable frame par frame »,
`DetectLines` détenu en `const DetectLines&`), `LaneTracker` n'est **pas**
possédé par `DetectLines` : il est construit dans `main.cpp` et possédé par
`PipelineRunner`, exactement comme `DetectLines` l'est aujourd'hui.
`DetectLines` reste inchangé, stateless, `const`.

Nouveau flux dans `PipelineRunner::process_frame` :

```cpp
const LaneModel raw_model = m_detector.compute( p_frame );
const LaneModel model = m_tracker.update( raw_model );   // nouveau

// render(), notification des observateurs : inchangé, mais avec `model`
// (lissé) au lieu de `raw_model`.
```

`compute_ms` continue d'englober l'appel à `update()` (coût négligeable : une
poignée de flops, pas de nouveau chrono dédié).

`PipelineRunner` gagne un paramètre de construction `LaneTracker& p_tracker`
(non-const : `update()` mute l'état interne), au même niveau que
`p_frame_source`/`p_detector`/`p_observers`.

## Interface

```cpp
class LaneTracker
  {
  public:
    LaneTracker( const VideoCaracteristics& p_video,
                 const LaneConfig& p_config,
                 ImageSink& p_debug_sink );

    /// @brief Lisse un LaneModel brut à travers le temps.
    /// @param p_raw_model Modèle de la frame courante (sortie de DetectLines::compute).
    /// @return Modèle lissé (ou p_raw_model tel quel si le tracker est désactivé
    /// par configuration, ou si aucun état exploitable n'existe encore).
    LaneModel update( const LaneModel& p_raw_model );

    /// @brief Réinitialise l'état interne. Jamais appelé par PipelineRunner en
    /// usage normal (une instance vit le temps d'un run) ; utilitaire de test.
    void reset();

  private:
    const VideoCaracteristics m_video_properties;
    const LaneConfig m_config;
    ImageSink& m_debug_sink;

    LanePolynomial m_smoothed_left;
    LanePolynomial m_smoothed_right;
    bool m_has_state = false;
    int m_miss_streak = 0;
  };
```

## Algorithme

### Activation/désactivation

`update()` teste `m_config.lane_tracker_enabled` en tout début de méthode. Si
`false` : retourne `p_raw_model` inchangé, sans toucher à `m_has_state` ni
`m_miss_streak`. `PipelineRunner` appelle `update()` **uniformément**, que le
tracker soit activé ou non — pas de `if` côté orchestration, même logique que
`--record` qui gouverne le rendu uniformément pour les 3 modes sans cas
particulier.

### Détection fraîche (`p_raw_model.lane_detected == true`)

1. `m_miss_streak = 0`.
2. Si `!m_has_state` (premier appel, ou reprise après un reset) : seed
   identité — `m_smoothed_left = p_raw_model.left`,
   `m_smoothed_right = p_raw_model.right`, `m_has_state = true`. Pas de mélange
   avec un état nul/périmé.
3. Sinon, moyenne mobile exponentielle (EMA) par coefficient, indépendamment
   sur les deux côtés :

   ```
   smoothed.a = alpha * raw.a + (1 - alpha) * smoothed_prev.a
   smoothed.b = alpha * raw.b + (1 - alpha) * smoothed_prev.b
   smoothed.c = alpha * raw.c + (1 - alpha) * smoothed_prev.c
   ```

   où `alpha = m_config.lane_tracker_alpha`.
4. `LaneGeometry::compute` est **rappelé** sur un `LaneModel` temporaire
   construit à partir de `m_smoothed_left`/`m_smoothed_right` (tous deux
   `valid = true`) pour régénérer `offset`/`normalized_offset`/
   `curvature_radius_px`/`lane_detected` de façon cohérente — pas de
   duplication de la formule de courbure/offset.
5. `reconstructed` propage `p_raw_model.reconstructed` (signal dégradé par
   reconstruction latérale, orthogonal au lissage). `coasted = false`.

### Détection manquée (`p_raw_model.lane_detected == false`)

1. `++m_miss_streak`.
2. Si `m_has_state && m_miss_streak <= m_config.lane_tracker_max_coast_frames` :
   **coasting** — reconduit le dernier modèle lissé connu (même
   `m_smoothed_left`/`m_smoothed_right`, géométrie déjà cohérente).
   `coasted = true`.
3. Sinon (perte trop longue, ou pas encore de state) : `m_has_state = false`
   (reset dur — le prochain fit valide reseed à l'identité plutôt que de
   mélanger avec un état périmé), retourne `p_raw_model` tel quel
   (`lane_detected = false`, `coasted = false`).

### Image fixe : comportement garanti identique

Une image fixe ne produit qu'un seul appel à `update()`. `m_has_state` est
`false` au départ, donc :

- Si détectée : branche « seed identité » — `m_smoothed_* = raw.*` (copie
  exacte). Le recalcul de `LaneGeometry::compute` sur des coefficients
  identiques aux originaux redonne exactement les mêmes doubles en sortie
  (fonction pure, pas de dérive flottante possible). Résultat bit-identique à
  l'absence de tracker.
- Si non détectée : branche « pas encore de state » — retour de
  `p_raw_model` tel quel.

Ce comportement découle des règles générales ci-dessus, sans cas particulier
codé pour le mode image — cohérent avec le principe déjà en vigueur dans le
projet (`--record` : « la still image reste un cas dégénéré du flux, pas une
exception »).

## Configuration (`LaneConfig`)

```cpp
// --- Lissage temporel (LaneTracker) ---
bool lane_tracker_enabled = true;          ///< false = update() est un pass-through.
double lane_tracker_alpha = 0.3;           ///< Poids de la mesure fraiche dans l'EMA (0 < alpha <= 1).
int lane_tracker_max_coast_frames = 10;    ///< Frames de coasting tolerees avant reset (~0.3s a 30 fps).
```

Valeurs par défaut à considérer comme point de départ, à caler sur piste
réelle via la trace de debug (cf. plus bas) — même logique que les seuils
`LaneMask`/`SlidingWindowSearch` existants.

## `LaneModel` — nouveau champ

```cpp
bool coasted = false;  ///< true si ce frame n'a pas de detection fraiche
                        ///< (dernier modele lisse reconduit par le tracker).
```

Distinct de `reconstructed` (spécifique à la reconstruction latérale d'un
côté manquant par décalage — orthogonal au lissage temporel).

### Impact CSV (`stdout`)

Nouvelle colonne **en fin de ligne**, pour ne pas décaler un éventuel
parseur positionnel existant sur les colonnes actuelles :

```
frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms
```

CLAUDE.md et le README (section format CSV) sont à mettre à jour en
conséquence dans le cadre de l'implémentation.

## Debug trace

`out/debug_04c_tracker.jpg` (après `debug_04_fit.jpg`/`debug_04b_quality.jpg`,
avant `debug_05_overlay.jpg`, cohérent avec la numérotation déjà en place) :
polylines brutes (rouge/bleu, comme `debug_04_fit.jpg`) superposées aux
polylines lissées (couleurs distinctes), plus un texte indiquant `alpha`,
`miss_streak` courant et `coasted`. Écrite par `LaneTracker::update` si
`m_debug_sink.is_enabled()`, uniformément que le tracker soit activé ou non
(si désactivé, brut et lissé coïncident exactement — information utile en soi).

`main.cpp` passe le même `ImageSink` de debug (`DiskImageSink`/`NullImageSink`,
choisi selon `LINE_DETECTOR_DEBUG`) à `LaneTracker` qu'à `DetectLines`.

## Tests prévus

- `test_lane_tracker.cpp` (nouveau, unitaire) :
  - seed identité au premier appel (sortie == entrée) ;
  - convergence EMA sur une séquence de fits synthétiques qui dérivent
    progressivement ;
  - coasting : N frames de miss consécutifs reconduisent le dernier modèle
    lissé, `coasted = true` ;
  - reset après dépassement de `max_coast_frames` : `lane_detected = false`
    renvoyé tel quel, et le fit valide suivant reseed à l'identité (pas de
    mélange avec l'état périmé) ;
  - `lane_tracker_enabled = false` : pass-through strict, aucun état modifié.
- `test_pipeline_runner.cpp` (mise à jour) : vérifie que `PipelineRunner`
  appelle le tracker et que les stats (`detected_count`, etc.) reflètent le
  modèle lissé.
- `tests/test_video_integration.cpp` (existant, non-régression) : la dérive
  monotone de `normalized_offset` doit rester vérifiée avec le lissage actif
  (une EMA préserve la monotonie d'une dérive lente ; à confirmer par
  l'exécution du test, pas juste par argument).

## Alternatives considérées

- **Filtre de Kalman** (mentionné dans CLAUDE.md comme piste) : modèle d'état
  position/vitesse par coefficient, fusion statistiquement principiée du bruit
  de mesure/process. Écarté pour cette première itération : complexité
  d'implémentation et de calibration nettement supérieure (6-8 paramètres de
  bruit vs 1 seul `alpha`) pour un gain incertain tant que l'EMA n'a pas
  montré ses limites en usage réel. Une évolution vers Kalman reste possible
  plus tard sans changer l'interface `LaneTracker::update()`.
- **Lissage du signal final uniquement** (offset/courbure, sans toucher aux
  polynômes) : écarté — l'overlay aurait dessiné des polynômes bruts (jitter
  visuel) pendant que le HUD affichait des valeurs lissées, incohérence
  visible en mode debug/`--record`.

## Risques / limites connues

- `alpha`/`max_coast_frames` sont des valeurs de départ, à valider sur piste
  réelle (pas de données d'enregistrement disponibles au moment de cette
  spec).
- Le recalcul de `LaneGeometry::compute` après lissage double son coût par
  frame détectée (négligeable en pratique : quelques flops).
- Un coasting prolongé (proche de `max_coast_frames`) peut reconduire un
  signal devenu faux si le véhicule a réellement dévié pendant la perte de
  détection — compromis assumé, cohérent avec le choix de garder l'EMA simple
  pour cette itération.
