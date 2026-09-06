# CLAUDE.md

Ce fichier fournit des repères pour travailler sur le code de ce dépôt.

## Présentation

Bibliothèque de suivi de lignes au sol en C++17, basée sur OpenCV, pensée pour
l'embarqué sur véhicule miniature autonome (caméra Raspberry Pi). En **usage
nominal**, l'entrée est une image ou une vidéo et la sortie est un **signal de
pilotage** (offset latéral normalisé + rayon de courbure) destiné à maintenir
le véhicule entre les lignes — pas d'image annotée en sortie. Un second usage,
dédié au **débug des algorithmes**, produit en plus des images annotées (cf.
`--record` et `LINE_DETECTOR_DEBUG` plus bas). Le pipeline **vue de dessus
(bird's eye view) + ajustement polynomial** permet de suivre des lignes
**courbes**. `main.cpp` traite une image fixe, un fichier vidéo, ou un flux
caméra en direct (`--image` / `--video` / `--camera`, cf. Compilation &
exécution).

**Portée volontairement générique** : cette lib doit rester utilisable par
d'autres projets que celui de l'auteur, indépendamment de tout véhicule ou
pipeline logiciel particulier. Elle est développée en vue d'un usage sur piste
d'athlétisme (véhicule suiveur de ligne), et l'auteur prévoit à terme de
l'intégrer dans un projet personnel de robot pacer, possiblement avec ROS2 —
mais ni ce projet aval ni ROS2 ne doivent influencer les choix de conception
de la lib (pas de dépendance ROS2, pas d'API pensée pour un seul
consommateur). `line_detector_lib` reste indépendante de tout framework
applicatif, cf. « Couche application ».

**Mode vidéo** : le mode vidéo (fichier + caméra) lit et traite frame par
frame, et lisse le `LaneModel` à travers les frames via `LaneTracker` (EMA sur
les coefficients des polynômes de voie + coasting sur perte de détection
courte), activable/désactivable par `LaneConfig::lane_tracker_enabled` — cf.
`docs/superpowers/specs/2026-09-06-lane-tracker-design.md`. Restent au
programme : recherche autour du fit précédent, correction de distorsion
caméra, passage à une sortie métrique (cf. section « Suite prévue »).

## Compilation & exécution

Build CMake hors-source. **OpenCV n'est pas installé sur l'hôte de dev (macOS)** :
la compilation se fait dans le conteneur Docker (image `line-detector`, base
`debian:bookworm-slim` + `libopencv-dev`). L'image se construit avec
`docker build -t line-detector -f docker/Dockerfile .`.

Compiler l'exécutable et lancer sur une image, en montant la source :

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector -j \
           && mkdir -p /app/out && cd /app && /tmp/build/line_detector --image img_piste/img2.jpg --record'
```

Trois modes, mutuellement exclusifs : `--image <chemin>` (défaut : `img_piste/img2.jpg`),
`--video <chemin>` (fichier vidéo) et `--camera [index]` (défaut `0`). Avec `--record`,
le mode image écrit `out/output.jpg` ; les modes flux écrivent `out/output.avi` — la
vidéo annotée est toujours écrite à une cadence **fixe de 30 fps**, quelle que soit la cadence
réelle de la source (`FrameSource` n'expose volontairement pas de `fps()`, cf.
Architecture) ; une vidéo issue d'une source plus lente ou plus rapide que 30 fps
paraîtra donc accélérée ou ralentie à la relecture — ce n'est pas un bug.

**`--record` gouverne l'écriture du résultat, uniformément pour les trois modes**
(y compris `--image` : la still image reste un cas dégénéré du flux, pas une
exception). Sans `--record`, le programme n'écrit **aucun fichier** — ni
`out/output.jpg`, ni `out/output.avi` — et n'exécute **aucun rendu** : aucun
observateur ne réclamant l'image annotée, `render` n'est jamais appelé
(`compute_ms`/`render_ms` le montrent : `render_ms` reste à `0`). Le message
`Resultat ecrit dans : …` n'apparaît que si `--record` est passé.

**`stdout` / `stderr` sont séparés** : `stdout` ne porte **que** la ligne d'en-tête
puis une ligne CSV par frame
(`frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms`,
format `std::fixed`, jamais de notation scientifique) ; `stderr` porte tous les
messages destinés à un humain (erreurs, résumé final frames/détections/ms/FPS —
dont la part de rendu entre parenthèses —, chemin du résultat écrit si
`--record`). Rediriger `stdout` seul suffit donc à consommer le
signal de pilotage sans parser de texte. **`render_ms` n'est pas « le coût de
production du fichier de sortie »** : avec `LINE_DETECTOR_DEBUG` **et**
`--record`, il inclut l'écriture de `debug_05_overlay.jpg` (cet `imwrite` a
lieu dans `LaneOverlay::render`, donc dans `DetectLines::render`), alors que
l'encodage du fichier résultat lui-même (`out/output.jpg`/`.avi`) se fait dans
l'observateur (`ResultImageWriter`/`AnnotatedVideoWriter`), hors des deux
chronomètres. `Ctrl-C` arrête proprement la boucle et
ferme le fichier vidéo ; un second `Ctrl-C` termine toujours le processus, même si
la lecture caméra est bloquée.

Sur une machine où OpenCV est installé au niveau système, le build générique
fonctionne aussi (`mkdir build && cd build && cmake .. && make`) ; l'exécutable
s'appelle `line_detector` (`add_executable(line_detector ...)` dans
`CMakeLists.txt`).

## Tests

Il existe une suite de tests unitaires **doctest** (header unique vendored dans
`tests/doctest.h`). Cible CMake séparée `line_detector_tests` ; les fichiers
`tests/test_*.cpp` sont ramassés par `file(GLOB ...)` (un nouveau test est pris en
compte car le build reconfigure via `cmake -S -B`). Lancer :

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```

Chaque composant du pipeline a ses tests ; deux tests bout-en-bout
(`tests/test_integration.cpp`) exercent tout `DetectLines` sur des images
synthétiques (dont une courbe qui vérifie que le fit n'est pas aplati).

Côté application, chaque composant a aussi sa suite : `test_cli_options.cpp`
(analyse des arguments, modes et erreurs), `test_frame_source.cpp`
(`StillImageFrameSource` et `CaptureFrameSource`), `test_frame_observer.cpp`
(`LaneModelLogger` et `ResultImageWriter`), `test_annotated_video_writer.cpp`
(écrit un fichier vidéo puis le relit avec `cv::VideoCapture`) et
`test_pipeline_runner.cpp` (boucle, stats, arrêt anticipé sur drapeau ou sur
échec définitif d'un observateur). `tests/test_video_integration.cpp` est le
pendant bout-en-bout du mode flux : une voie qui dérive progressivement passée
dans `PipelineRunner` complet, qui vérifie que `normalized_offset` évolue de
façon monotone — la preuve que le mode vidéo *suit* quelque chose, pas
seulement qu'il tourne sans planter. Pas de linter ni de CI.

## Architecture

Le pipeline est orchestré par `DetectLines` (`DetectLines.cpp`), qui expose deux
points d'entrée. **`compute()`** exécute les étapes 1 à 5 ci-dessous et **renvoie
un `LaneModel`** (le signal de pilotage) sans rien dessiner. **`render()`** prend
un `LaneModel` déjà calculé et une image, exécute l'étape 6 (`LaneOverlay`) et
dessine le résultat dans l'image de sortie. Étapes, dans l'ordre :

1. **`LaneMask`** (`LaneMask.cpp`) — image binaire des marquages depuis le BGR :
   blanc (seuil de luminance sur `COLOR_BGR2GRAY`), jaune (`inRange` en HSV),
   bords (Sobel x), combinés par OU binaire. Remplace l'ancien trio
   grayscale/flou/Canny. **Utilise `COLOR_BGR2GRAY`** (`imread` charge en BGR).
2. **`PerspectiveView::toBev`** (`PerspectiveView.cpp`) — warp perspective vers une
   vue de dessus via `getPerspectiveTransform`. Le quadrilatère source (trapèze
   dérivé de `VideoCaracteristics` + `LaneConfig`) hérite de la géométrie de
   l'ancien `RegionOfInterest` ; `warpBack` fait la transformation inverse.
3. **`SlidingWindowSearch::search`** (`SlidingWindowSearch.cpp`) — histogramme des
   colonnes (moitié basse) pour trouver les deux bases, puis fenêtres glissantes
   de bas en haut collectant les pixels gauche/droite (`LanePixels`). Une fenêtre
   sans assez de pixels ne se recentre pas.
4. **`LanePolynomial::fit`** (`LanePolynomial.cpp`) — ajuste `x = a·y² + b·y + c`
   par moindres carrés (`cv::solve`, Vandermonde) pour chaque côté.
5. **`LaneGeometry::compute`** (`LaneGeometry.cpp`) — remplit le `LaneModel` :
   centre de voie, offset latéral, offset normalisé, rayon de courbure. Reconstruit
   un côté manquant par décalage (cf. plus bas).
6. **`LaneOverlay::render`** (`LaneOverlay.cpp`) — remplit le polygone de voie en
   BEV, le ramène en perspective image (`warpBack`), le fusionne sur l'image
   d'origine et ajoute un HUD (offset + courbure). **Contrairement aux étapes 1
   à 5, qui appartiennent à `DetectLines::compute`, cette étape ne s'exécute que
   dans `DetectLines::render`** — elle est sautée si aucun observateur ne
   réclame l'image annotée (cf. « Couche application »).

`LaneTracker` (`LaneTracker.cpp`) lisse le `LaneModel` entre les frames (EMA
sur les coefficients de `LanePolynomial`, coasting sur perte de détection
courte). Ce n'est **pas** une étape de `DetectLines::compute`/`render` : c'est
le seul composant de `line_detector_lib` qui porte un état entre deux appels,
et pour cette raison il n'est pas possédé par `DetectLines` (qui reste sans
état, cf. « Couche application » ci-dessous) mais par `PipelineRunner`, qui
appelle `update()` juste après `compute()` et avant `render()`.

Types clés et possession :

- **`VideoCaracteristics`** (`VideoCaracteristics.h`) — struct contenant
  `image_size`, `width_pixel`, `height_pixel`, construite depuis un `cv::Mat` de
  référence. Source unique de la géométrie ; propagée par valeur const à chaque
  composant. La créer en premier.
- **`LaneConfig`** (`LaneConfig.h`) — struct centralisant **tous** les réglages
  (seuils blanc/jaune/Sobel, ratios de calibration BEV, fenêtres glissantes,
  `defaultLaneWidthPx`). Construite dans `main`, injectée par valeur const. C'est
  le point de réglage unique (notamment la calibration perspective).
- **`LaneModel`** (`LaneModel.h`) — résultat : deux `LanePolynomial` (gauche/droite,
  en pixels BEV), `laneDetected`, le signal (`lateralOffsetPx`,
  `normalizedOffset`, `curvatureRadiusPx`) et `reconstructed`. **Signe de
  l'offset : négatif = véhicule décalé à gauche.**
- **`LanePolynomial`** (`LanePolynomial.h`) — polynôme `x = a·y² + b·y + c` avec un
  drapeau `valid` et une fabrique statique `fit(points, minPoints)`.
- **`DetectLines`** possède `LaneMask mask`, `PerspectiveView perspective`,
  `SlidingWindowSearch search`, `LaneOverlay overlay` — tous initialisés depuis la
  même `VideoCaracteristics`/`LaneConfig`. **`overlay` est déclaré après
  `perspective`** car il détient une `const PerspectiveView&` (l'ordre de
  déclaration = ordre de construction).
- **`projectTypes.h`** — `DimensionImage` est un `typedef uint16_t`, utilisé pour
  les dimensions en pixels.
- **`ImageSink`** (`ImageSink.h`) — interface `bool save(name, frame)`.
  `DiskImageSink` écrit dans un dossier (`cv::imwrite`), `NullImageSink` est un
  no-op. Résultat final et traces de debug passent par la même interface, mais
  ne sont **plus** garantis d'exister : le sink de résultat n'est construit
  (`DiskImageSink`) que si `--record` est passé, sinon aucun observateur
  d'écriture n'est même créé ; les traces sont `DiskImageSink` si
  `LINE_DETECTOR_DEBUG` est défini, sinon `NullImageSink`. Les sinks sont créés
  dans `main` et injectés par référence dans `DetectLines` puis les composants.

Ordre de construction dans `main.cpp` : `VideoCaracteristics` depuis la
**première frame lue**, puis `LaneConfig`, puis `DetectLines`. `main` n'appelle
ni `compute` ni `render` lui-même : il assemble aussi la `FrameSource` et les
`FrameObserver`, puis délègue la boucle à `PipelineRunner::run`, qui appelle
`DetectLines::compute` frame par frame, puis `DetectLines::render` seulement si
au moins un observateur réclame l'image annotée, et notifie les observateurs
avec le `LaneModel` renvoyé (cf. « Couche application » ci-dessous).

### Couche application

Deux cibles CMake matérialisent la frontière :

- **`line_detector_lib`** — le pipeline de détection ci-dessus (étapes 1 à 6 +
  `ImageSink`). **Règle de frontière, vérifiable en revue :** rien dans
  `line_detector_lib` ne lit `argv`, n'écrit sur `stdout`, ni ne possède de boucle
  de capture.
- **`line_detector_app`** — le harnais : capture, boucle, sorties dev/test. Lie
  `line_detector_lib`. C'est cette cible que `line_detector` (l'exécutable,
  `main.cpp` seul) **et** `line_detector_tests` linkent, pour garantir qu'ils
  exercent exactement le même code.

Trois en-têtes purement utilitaires — `VideoCaracteristics.h`, `projectTypes.h`,
`SmartAssert.h` — sont utilisés à la fois par `line_detector_lib` et
`line_detector_app` (et par `main.cpp`) ; ils vivent dans **`src/common/`**,
pas dans `src/lib/`, pour rendre cette frontière visible dans l'arborescence.
`src/common/` n'a pas de cible CMake propre (en-têtes seuls, sans `.cpp`) : il
est simplement ajouté aux `target_include_directories` de `line_detector_lib`,
et propagé à `line_detector_app` par l'édition de liens `PUBLIC` existante.

Composants de `line_detector_app` :

- **`CliOptions` / `parse_arguments`** (`CliOptions.h/.cpp`) — analyse pure des
  arguments (`argc`/`argv` → `CliOptions`), testable sans lancer de processus.
  Trois modes mutuellement exclusifs : `--image <chemin>`, `--video <chemin>`,
  `--camera [index]` (`e_source_kind`). Une valeur manquante ou qui ressemble à
  un flag après `--image`/`--video` est rejetée (`ERROR_MISSING_VALUE`).
- **`FrameSource`** (`FrameSource.h`) — interface minimale, un seul `read`.
  Deux implémentations : **`StillImageFrameSource`** (une image `cv::imread`,
  rendue une fois — le mode image fixe est un cas dégénéré du mode vidéo) et
  **`CaptureFrameSource`** (enveloppe `cv::VideoCapture`, fabriques statiques
  `from_file`/`from_camera`, non copiable).
- **`FrameObserver`** (`FrameObserver.h`) — interface `on_frame(index, model,
  annotated_frame, compute_ms, render_ms)`, notifiée à chaque frame par
  `PipelineRunner`. Trois implémentations : **`LaneModelLogger`** (une ligne CSV
  par frame sur un `std::ostream` injecté — `main` lui donne `std::cout`),
  **`ResultImageWriter`** (mode `--image`, écrit via un `ImageSink`) et
  **`AnnotatedVideoWriter`** (modes flux, `cv::VideoWriter` ouvert paresseusement
  à la première frame, non copiable). Chaque implémentation qui écrit expose
  `has_fatal_error()` (par défaut `false` dans l'interface), qui reflète son
  `has_failed()`. Chaque observateur expose aussi `needs_annotated_frame()`
  (défaut `true` dans l'interface ; redéfini à `false` par `LaneModelLogger`,
  qui n'exploite que le `LaneModel`). `PipelineRunner` calcule une seule fois, à
  la construction, si **au moins un** observateur en a besoin ; si aucun n'en a
  besoin (typiquement : logger seul, sans `--record`), `annotated_frame` reçu
  par `on_frame` est un `::cv::Mat` **vide** et `DetectLines::render` n'est
  jamais appelé — un observateur qui rend `false` ne doit pas déréférencer ce
  `Mat`. Conséquence en debug (`LINE_DETECTOR_DEBUG`) : `debug_01_mask.jpg` à
  `debug_04_fit.jpg` sont toujours écrits (ils viennent de `compute`), mais
  `debug_05_overlay.jpg` — écrit dans `LaneOverlay::render`, donc dans
  `DetectLines::render` — ne l'est **que si `--record` est passé**.
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

## Design by contract

Distinction stricte, appliquée dans tout le pipeline :

- **Violation d'invariant de code** (ne devrait jamais arriver si le code est
  correct : `Mat` vide, mauvais nombre de canaux, taille ≠ `VideoCaracteristics`,
  homographie dégénérée, offset non fini) → **`SMART_ASSERT`** (`SmartAssert.h`),
  macro **toujours active** (non supprimée par `NDEBUG`, contrairement à `assert`).
- **Aléa de la route** (voie absente, trop peu de pixels, largeur non plausible)
  → **drapeau** (`LanePolynomial::valid`, `LaneModel::laneDetected`), jamais un
  assert. Un côté manquant est reconstruit par décalage (`LaneModel::reconstructed`
  passe à `true` — signal dégradé, à signaler au module de contrôle).
- **Aléa d'environnement, hors pipeline** (uniquement dans `main.cpp` : arguments
  CLI invalides, source introuvable ou indisponible, dossier de sortie
  inutilisable, échec d'écriture du résultat) → **`EXIT_IF_FAILED`**
  (`SmartAssert.h`, même fichier que `SMART_ASSERT` mais macro **distincte** et
  volontairement différente : message humain sur `stderr` (pas de fichier:ligne
  ni de condition brute — ce message s'adresse à l'utilisateur, pas au
  développeur) puis `exit(EXIT_FAILURE)` propre, jamais `abort()`. Ce n'est pas
  un bug à corriger mais une situation normale (fichier absent, disque plein,
  caméra débranchée) à rapporter sans faire planter le programme. Réservé à
  `main.cpp` : `line_detector_lib`/`line_detector_app` ne terminent jamais le
  process elles-mêmes (cf. « Couche application » ci-dessous).

## Conventions & pièges

- **Tout le code doit respecter `docs/coding_rules.md`** (nommage et
  organisation des fichiers, style d'indentation, conventions de nommage et
  d'écriture). C'est la référence de style du projet, à appliquer à tout code
  nouveau ou modifié.
- Le code, les commentaires et les messages de commit mélangent **français et
  anglais** ; suivre la langue du fichier environnant.
- **Les commits doivent respecter le skill `/git-commit`** (message de type
  commit conventionnel généré à partir du diff, staging intelligent par
  regroupement logique).
- Le **nouveau code qualifie explicitement `cv::`**. Certains en-têtes hérités
  contiennent encore `using namespace cv;`.
- Les réglages ne sont plus des consts éparpillées : ils vivent dans **`LaneConfig`**.
- `main.cpp` n'est plus qu'un assemblage : il analyse les arguments (`parse_arguments`),
  construit la `FrameSource`, lit la **première frame** (c'est elle qui définit
  `VideoCaracteristics`, jamais les métadonnées de `cv::VideoCapture`), construit le
  détecteur et les observateurs, puis délègue la boucle à `PipelineRunner`. La
  bibliothèque `line_detector_lib` ne lit pas `argv`, n'écrit pas sur `stdout` et ne
  possède aucune boucle : `cv::VideoCapture` vit côté application.
- **Dossier de sortie** : par défaut `out`, surchargeable par la variable d'environnement
  `LINE_DETECTOR_OUT`. `cv::imwrite`/`cv::VideoWriter` ne créent pas ce dossier :
  `main` le crée lui-même (`std::filesystem::create_directories`), mais **seulement
  si quelque chose sera effectivement écrit** — `--record`, ou `LINE_DETECTOR_DEBUG`
  (cf. « Couche application »). Sans l'un ou l'autre, aucun dossier n'est créé ni
  validé : un `out/` inutilisable n'est alors pas une raison d'échec, puisque rien
  n'en a besoin — utile en particulier au premier lancement d'une caméra sans
  `--record`.
- **Traces de debug en mode flux** : les noms de fichiers sont fixes, donc chaque frame
  écrase la précédente — la dernière frame gagne. Voulu : les traces servent à caler la
  calibration BEV, pas à archiver la séquence.
- **Calibration BEV** : les 4 points source dérivent des ratios
  `src_top_width_ratio` / `src_top_y_ratio` (bord haut) et `src_bottom_width_ratio` /
  `src_bottom_y_ratio` (bord bas) de `LaneConfig`. Par défaut le bord bas colle aux
  coins de l'image (`src_bottom_y_ratio=1.0`, `src_bottom_width_ratio=0.5`), ce qui
  suppose que les deux marquages restent dans le cadre jusqu'au bas de l'image —
  vrai pour une caméra montée haut (~1 m). Pour une caméra très basse (quelques cm
  du sol), les marquages sortent du cadre par les côtés avant d'atteindre le bas :
  remonter `src_bottom_y_ratio` et réduire `src_bottom_width_ratio` pour que le bord
  bas du trapèze reste dans la zone où les deux lignes sont encore visibles. Régler
  en inspectant `out/debug_02a_trapeze.jpg` (trapèze superposé à l'image source) et
  `out/debug_02_bev.jpg` jusqu'à ce que des lignes droites parallèles apparaissent
  verticales et parallèles. **Non bloquant** pour une caméra haute : les valeurs par
  défaut fonctionnent (précision géométrique dégradée seulement).
- **Traces de debug** : exécuter avec `LINE_DETECTOR_DEBUG` non vide
  (`LINE_DETECTOR_DEBUG=1 ./line_detector`) écrit les étapes intermédiaires :
  `out/debug_01_mask.jpg`, `debug_02_bev.jpg`, `debug_03_windows.jpg`,
  `debug_04_fit.jpg` et `debug_04c_tracker.jpg` (polynômes bruts vs lissés par
  `LaneTracker`) sont écrits dès que la variable est définie ; `debug_05_overlay.jpg`
  demande **en plus** `--record` (il vient de `render`, cf. « Couche application »).
  Sans la variable, aucune trace (`NullImageSink`). Choix **runtime** : pas
  d'option CMake.
- **Conteneurisation** : `Dockerfile` (base `debian:bookworm-slim`) installe
  OpenCV via apt et compile le projet. `tools/make_test_image.py` (Pillow) génère
  des images de test dans `img_piste/` : `img2.jpg`, `straight.jpg`, `curved.jpg`,
  `shifted.jpg`, `dashed.jpg`.
- **Code retiré** : l'ancien pipeline Canny/Hough (`grayscal`, `median_blur`,
  `canny_edge_detection`, `hough_lines`, `compute_angle_from_two_points`) et la
  classe `RegionOfInterest` (son trapèze est absorbé par `PerspectiveView`). Le
  code caméra (`cv::VideoCapture`) avait été retiré puis **réintroduit** côté
  application par `CaptureFrameSource` (mode `--camera`) : c'est une des
  fonctionnalités phares de ce lot vidéo.

## Suite prévue

Voir `docs/superpowers/specs/2026-09-06-lane-tracker-design.md` pour le design
du lissage temporel. Le mode vidéo (fichier + caméra, cf. Couche application
ci-dessus) et le lissage temporel (`LaneTracker`) sont faits ; reste au
programme : recherche autour du fit précédent, correction de distorsion
caméra, passage à une sortie métrique.
