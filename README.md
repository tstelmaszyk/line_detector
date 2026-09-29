# line-detector

Bibliothèque de suivi de lignes au sol en C++17, basée sur OpenCV. Elle prend
une image de route (caméra embarquée sur un véhicule miniature autonome) et en
tire un signal de pilotage : de combien le véhicule est décalé par rapport au
centre de la voie, et quel est le rayon de courbure de la voie devant lui. Pas
de réseau de neurones, pas de dépendance à un framework robotique : uniquement
du traitement d'image classique (seuillage couleur, transformée perspective,
ajustement polynomial), ce qui la rend prévisible, débogable à l'image près, et
assez légère pour tourner en temps réel sur un Raspberry Pi.


## Table des matières

- [Ce que fait le programme](#ce-que-fait-le-programme)
- [Le pipeline, étape par étape](#le-pipeline-étape-par-étape)
- [Les algorithmes en détail](#les-algorithmes-en-détail)
- [Architecture du code](#architecture-du-code)
- [Structure du dépôt](#structure-du-dépôt)
- [Prérequis](#prérequis)
- [Installation et exécution](#installation-et-exécution)
- [Utilisation du programme](#utilisation-du-programme)
- [Traces de debug](#traces-de-debug)
- [Tests](#tests)
- [Limites connues](#limites-connues)

---

## Ce que fait le programme

Deux usages coexistent dans le même exécutable.

**Usage nominal** : `line_detector` lit une frame, calcule un `LaneModel`
(offset latéral normalisé + rayon de courbure) et l'écrit en CSV sur `stdout`,
une ligne par frame. C'est ce flux qu'un module de contrôle en aval
consommerait pour piloter le véhicule. Aucune image n'est produite dans ce
mode : le calcul du signal ne dépend d'aucun rendu.

**Usage debug** : avec `--record`, le programme dessine en plus le résultat
sur l'image d'origine (la voie détectée en surimpression, un HUD texte) et
l'écrit sur disque. Avec la variable d'environnement `LINE_DETECTOR_DEBUG`,
il écrit aussi les étapes intermédiaires du pipeline (masque, vue de dessus,
fenêtres de recherche, polynômes ajustés) — utile pour calibrer les seuils et
la perspective sur une caméra donnée.

Le programme traite une image fixe (`image --file`) ou un flux
(`video --file` pour un fichier vidéo, `video --camera` pour une webcam,
`video --gstreamer` pour un pipeline GStreamer, par exemple la caméra CSI d'un
Raspberry Pi 5). Le mode vidéo lit et traite frame par frame, et lisse le
`LaneModel` à travers les frames (`LaneTracker`, EMA sur les coefficients des
polynômes + coasting sur perte de détection courte). Il n'y a en revanche
toujours pas de recherche localisée autour du fit précédent (voir [Limites
connues](#limites-connues)).

La bibliothèque de détection elle-même est volontairement indépendante de tout
ce qui est spécifique à ce véhicule : elle prend une image, elle rend
un modèle de voie, point. Rien dans `line_detector_lib` ne lit `argv`, n'écrit
sur `stdout`, ni ne sait qu'une caméra existe.

## Le pipeline, étape par étape

```
   image caméra (BGR)
        │
        │  1. isoler les marquages           →  masque binaire noir & blanc
        ▼
   masque en perspective caméra
        │
        │  2. redresser (bird's eye view)    →  lignes parallèles et verticales
        ▼
   masque vu de dessus
        │
        │  3. suivre chaque ligne             →  nuage de pixels gauche / droite
        │  4. ajuster une courbe              →  x = a·y² + b·y + c, par côté
        │  5. vérifier la confiance du fit    →  démote un côté douteux (valid=false)
        │  6. en déduire la géométrie         →  offset + rayon de courbure
        ▼
   LaneModel (le signal de pilotage, sorti en CSV sur stdout)
        │
        │  7. rendu, seulement si demandé     →  overlay sur l'image d'origine
        ▼
   image annotée (out/output.jpg ou out/output.avi, avec --record)
```

Les étapes 1 à 6 sont regroupées dans `DetectLines::compute()`, qui ne dessine
rien et renvoie uniquement le `LaneModel`. L'étape 7, `DetectLines::render()`,
en est délibérément séparée : elle prend un `LaneModel` déjà calculé et une
image, et produit l'image annotée. Cette séparation est ce qui permet au mode
sans `--record` de sauter entièrement le rendu — `render_ms` reste à zéro,
aucun `cv::Mat` de sortie n'est même alloué.

## Les algorithmes en détail

### 1. Isoler les marquages (`LaneMask`)

On ne cherche pas des « lignes » au sens géométrique tout de suite — on
commence par isoler les pixels qui ressemblent à du marquage au sol, en
combinant trois heuristiques indépendantes par un OU binaire.

**Blanc** : on convertit l'image en niveaux de gris (`cvtColor` avec
`COLOR_BGR2GRAY`, en gardant à l'esprit que `cv::imread` charge en BGR et non
RGB) et on seuille la luminance à 200/255 (`white_threshold`). Un pixel
au-dessus du seuil est considéré comme marquage blanc. Simple, mais suffisant
sur un marquage peint contrastant avec l'asphalte.

**Jaune** : on convertit en HSV et on garde les pixels dont la teinte tombe
dans `[15, 35]` (sur l'échelle OpenCV 0–179), avec une saturation et une
valeur minimales de 80/255 chacune (`inRange`). Travailler en teinte plutôt
qu'en RGB rend le seuil robuste aux variations d'exposition — un jaune plus
sombre ou plus clair reste dans la même plage de teinte.

**Bords (gradient de Sobel)** : les deux masques couleur ratent les marquages
déteints ou les bords de chaussée sans peinture. On floute l'image en niveaux
de gris (`GaussianBlur`, noyau 5×5) pour atténuer le grain de l'asphalte, puis
on calcule le gradient horizontal par un filtre de Sobel (`Sobel(..., dx=1,
dy=0, ksize=3)`). Un marquage qui s'éloigne de la caméra produit une frontière
à peu près verticale dans l'image ; le gradient *horizontal* (dérivée selon x)
y est donc fort, alors qu'il reste faible sur la texture granuleuse de la
route. Le résultat est seuillé en amplitude (`convertScaleAbs` puis `inRange`
entre 80 et 150).

Le flou ne suffit pas à effacer complètement le bruit de texture dans le champ
proche (bas de l'image, où chaque pixel couvre une petite surface réelle) : le
masque Sobel y reste trop bruyant pour être utile. On le désactive donc dans
les `sobel_near_cutoff_ratio` (60 %) lignes les plus basses, et on ne le garde
que dans le champ lointain, plus flou par nature et donc moins texturé après
le `GaussianBlur`.

Les trois masques sont combinés par OU (`white | yellow | sobel_bin`), puis
une **ouverture morphologique** (érosion suivie d'une dilatation, noyau
elliptique 3×3) élimine les pixels isolés — du bruit sans cohérence spatiale —
sans ronger la largeur des vrais marquages.

### 2. Redresser en vue de dessus (`PerspectiveView`)

Le problème avec une image en perspective, c'est que deux lignes parallèles au
sol convergent visuellement vers un point de fuite : leur distance en pixels
n'a plus rien à voir avec leur distance réelle, et un polynôme ajusté dessus
serait faussé par cet effet purement optique. On corrige ça en amont, avant
tout ajustement de courbe.

On définit un trapèze dans l'image source — plus large en bas (proche du
véhicule) qu'en haut (loin) — dont les quatre coins sont dérivés de quatre
ratios de `LaneConfig` : `src_top_y_ratio` / `src_top_width_ratio` pour le
bord haut, `src_bottom_y_ratio` / `src_bottom_width_ratio` pour le bord bas,
chacun exprimé en fraction de la largeur/hauteur de l'image. Ce trapèze
représente la portion de route qu'on va « aplatir ». `cv::getPerspectiveTransform`
résout l'homographie (matrice 3×3, à un facteur d'échelle près) qui envoie
exactement ces quatre coins sur les quatre coins d'un rectangle plein cadre
(avec une marge latérale `bev_margin_ratio` pour laisser de la place aux
virages qui sortiraient du trapèze source). L'inverse est calculée
indépendamment (`getPerspectiveTransform` dans l'autre sens, pas une inversion
de matrice) pour ramener plus tard un résultat de la vue de dessus vers la
perspective d'origine.

`cv::warpPerspective` applique ensuite cette transformation projective à
chaque pixel : pour chaque position dans l'image de sortie, elle calcule la
position correspondante dans l'image source via la matrice homogène, et
interpole (`INTER_LINEAR`) entre les pixels voisins. Le résultat, bien réglé,
donne des marquages parallèles qui apparaissent verticaux et parallèles dans
la vue de dessus — c'est exactement le critère utilisé pour calibrer les
quatre ratios (cf. `out/debug_02_bev.jpg`, section debug plus bas).

Une homographie dégénérée (les quatre points source presque alignés)
produirait une matrice non inversible ; le déterminant est donc vérifié à la
construction (`SMART_ASSERT` — un déterminant proche de zéro ici serait un bug
de configuration, pas un aléa de la route).

### 3. Suivre chaque ligne (`SlidingWindowSearch`)

Une fois le masque redressé, il reste un nuage de pixels blancs plus ou moins
organisé en deux bandes verticales. L'algorithme des fenêtres glissantes
(*sliding window search*, popularisé par les pipelines de suivi de voie type
Udacity/OpenCV) les sépare en pixels gauche/droite sans jamais supposer que
les lignes sont droites.

**Amorce par histogramme.** On somme, colonne par colonne, le nombre de
pixels allumés sur une bande basse (les `histogram_band_ratio` = 25 %
dernières lignes de l'image). Le pic de cet histogramme sur la moitié gauche
de l'image donne la position de départ de la ligne gauche ; le pic sur la
moitié droite, celle de la ligne droite. C'est une hypothèse raisonnable :
juste devant le véhicule, les deux marquages sont presque toujours visibles et
denses.

**Fenêtres glissantes.** L'image est découpée en `window_count` (9) bandes
horizontales de hauteur égale. En partant de la bande la plus basse (donc la
plus proche du véhicule) et en remontant :

- pour chaque côté, on ouvre une fenêtre de largeur `2 × marge` centrée sur la
  position courante (`first_window_margin` = 100 px pour la toute première
  fenêtre, plus large pour absorber l'incertitude de l'amorce ; `window_margin`
  = 60 px ensuite) ;
- tous les pixels allumés dans cette fenêtre sont collectés dans le nuage de
  points de ce côté ;
- si la fenêtre contient plus de `window_min_pix` (50) pixels, son centre pour
  l'étage suivant devient la moyenne des abscisses des pixels trouvés — la
  fenêtre « suit » la ligne. Sinon, le centre ne bouge pas : une fenêtre
  vide ne dérive pas au hasard, elle reste sur la dernière position connue.

C'est ce recentrage qui permet de suivre une ligne qui courbe ou qui part sur
le côté, sans jamais ajuster de modèle géométrique à ce stade — seulement une
moyenne locale, étage par étage.

### 4. Ajuster une courbe (`LanePolynomial`)

Chaque nuage de points (gauche, droite) est ajusté par un polynôme du second
degré, mais paramétré par `y` plutôt que par `x` :

```
x = a·y² + b·y + c
```

Ce choix n'est pas arbitraire. En vue de dessus, une ligne de voie est
quasiment verticale — elle varie peu en x sur toute la hauteur de l'image. Un
polynôme `y = f(x)` classique aurait une pente proche de l'infini sur ce genre
de courbe (mal conditionné, voire indéfini sur une ligne parfaitement
verticale) ; `x = f(y)` n'a pas ce problème, puisque y balaie justement l'axe
le long duquel la ligne s'étend. C'est aussi ce degré 2 (au lieu d'un simple
segment de droite) qui permet de représenter un virage — une transformée de
Hough, qui ne produit que des droites, ne le pourrait pas.

L'ajustement se fait par moindres carrés. Pour N points `(x_i, y_i)`, on
construit une matrice de Vandermonde `M` (N lignes, 3 colonnes : `y_i²`, `y_i`,
`1`) et un vecteur `b` des abscisses observées, puis on résout le système
surdéterminé `M · [a, b, c]ᵀ ≈ b` au sens des moindres carrés via
`cv::solve(..., DECOMP_SVD)` — une décomposition en valeurs singulières, plus
robuste numériquement qu'une résolution par équations normales quand les
points sont mal répartis (par exemple concentrés sur une petite plage de y).
En dessous de `window_min_pix` points (50, le même seuil que pour recentrer
une fenêtre), l'ajustement est jugé non fiable : le polynôme reste
`valid = false` plutôt que de produire une courbe construite sur trop peu
d'information.

### 5. Vérifier la confiance du fit (`LaneQuality`)

Un fit peut réussir numériquement (assez de points pour `cv::solve`) tout en
étant géométriquement absurde — typiquement une caméra mal orientée, où le
masque capte du bruit des deux côtés qui converge ou diverge fortement plutôt
que de suivre deux marquages réellement parallèles. `LaneQuality` s'intercale
ici, avant que `LaneGeometry` ne calcule le signal, et démote
(`LanePolynomial::valid = false`) tout côté qui n'est pas digne de confiance,
selon deux critères indépendants :

- **Pixels insuffisants** — un côté dont `LanePolynomial::point_count` reste
  sous `min_quality_points` (150 par défaut, un seuil plus strict que
  `window_min_pix` qui ne garantit que la faisabilité numérique du fit, pas sa
  fiabilité) est démoté seul.
- **Largeur de voie incohérente** — si les deux côtés restent valides après le
  critère précédent, on échantillonne la largeur de voie
  (`x_right(y) - x_left(y)`) en bas, au milieu et en haut de la BEV. Si une
  largeur est quasi nulle ou négative (lignes croisées) ou si le ratio
  max/min dépasse `max_width_ratio_variation` (1.5 par défaut), **les deux**
  côtés sont démotés — impossible de savoir lequel des deux fits ment.

Le rejet reste un pur aléa de la route (drapeau, jamais un `SMART_ASSERT`) :
une image mal cadrée est une situation normale à signaler, pas un bug. Un côté
démoté redevient ensuite un côté manquant ordinaire pour `LaneGeometry`, qui
le reconstruit par décalage si `default_lane_width_px` est réglé, ou fait
retomber `lane_detected` à `false` si les deux côtés sont démotés. Aucun champ
n'est ajouté à `LaneModel` : le rejet reste visible uniquement via
`lane_detected = false`, exactement comme un côté manquant aujourd'hui.

### 6. En déduire le signal de pilotage (`LaneGeometry`)

C'est ici que les deux polynômes deviennent un offset et un rayon de courbure.

**Reconstruction d'un côté manquant.** Si un seul côté est valide (marquage
effacé, ligne discontinue temporairement hors champ) et que
`default_lane_width_px` est non nul (réglé dans `main.cpp` à 35 % de la
largeur de l'image), le côté manquant est reconstruit en copiant l'autre
polynôme et en décalant seulement son terme constant `c` de la largeur de voie
attendue — décaler `c` translate toute la courbe horizontalement sans changer
sa forme, puisque `a` et `b` gouvernent uniquement la courbure et la pente. Le
drapeau `reconstructed` passe alors à `true` : le signal reste exploitable
mais dégradé, à charge du consommateur d'en tenir compte. Si les deux côtés
manquent, ou qu'un seul manque sans largeur de secours configurée,
`lane_detected` passe à `false` et le signal est mis à zéro plutôt que de
propager une valeur inventée.

**Offset latéral.** Les deux polynômes sont évalués en bas de l'image
(`y_max = hauteur - 1`, la ligne la plus proche du véhicule) pour obtenir
`x_left` et `x_right`. Leur écart, `lane_width = x_right - x_left`, doit être
strictement positif et non négligeable (`> 1 px`) — une largeur nulle ou
négative signale une détection incohérente, pas une voie réelle, et fait
retomber sur `lane_detected = false`. Le centre de la voie est
`(x_left + x_right) / 2` ; en supposant le véhicule centré sur l'axe optique
de la caméra, l'écart entre le centre de l'image et le centre de la voie donne
l'offset :

```
lateral_offset_px   = centre_image_x − centre_voie
normalized_offset    = lateral_offset_px / (lane_width / 2)
```

La normalisation par la demi-largeur de voie rend la valeur comparable d'une
caméra à l'autre : `0` = parfaitement centré, `±1` = un bord de la voie
aligné avec le centre de l'image. Par convention, **un offset négatif signifie
que le véhicule est décalé à gauche** (le centre de voie apparaît alors à
droite du centre image).

**Rayon de courbure.** Pour une courbe `x(y) = a·y² + b·y + c`, la formule
classique du rayon de courbure d'une fonction paramétrée est :

```
R(y) = (1 + x'(y)²)^1.5 / |x''(y)|
```

Avec `x'(y) = 2a·y + b` et `x''(y) = 2a` (constant, puisque le polynôme est
d'ordre 2). Le calcul évalue la pente au même `y_max` que l'offset — le rayon
de courbure « au niveau du véhicule » — sur le polynôme gauche uniquement
(les deux côtés sont censés être à peu près parallèles une fois en vue de
dessus, donc l'un ou l'autre donne un rayon équivalent). Quand `|2a|` est
inférieur à `1e-9`, la voie est considérée droite au sens numérique et le
rayon est fixé arbitrairement à `1e12` px plutôt que de laisser une division
par une valeur proche de zéro produire un résultat instable.

### 7. Rendu (`LaneOverlay`, optionnel)

Cette étape ne s'exécute que si un observateur réclame l'image annotée (donc
seulement avec `--record`, cf. plus bas). Elle prend le `LaneModel` déjà
calculé — pas nécessairement celui de la frame en cours d'ailleurs, rien
n'empêche de dessiner un modèle lissé sur une frame plus récente — et :

1. évalue les deux polynômes à chaque ligne `y` de la vue de dessus pour
   construire un polygone fermé (le bord gauche du bas vers le haut, puis le
   bord droit du haut vers le bas) et le remplit en vert sur un canevas noir ;
2. ramène ce canevas dans la perspective d'origine via l'homographie inverse
   calculée à l'étape 2 (`warp_back`) ;
3. le fusionne sur l'image d'origine par pondération (`addWeighted`, poids
   1.0 pour l'image de base et 0.3 pour le calque de voie) — le calque étant
   noir partout sauf sur la voie, ceci teinte uniquement la zone détectée sans
   l'assombrir ailleurs ;
4. superpose un HUD texte (`offset=... R=...px`) en haut à gauche de l'image.

## Architecture du code

### La frontière bibliothèque / application

Le dépôt est scindé en deux cibles CMake, et cette frontière est vérifiable en
lisant le code, pas seulement documentée :

- **`line_detector_lib`** (`src/lib/`) est le pipeline décrit ci-dessus. Rien
  dedans ne lit `argv`, n'écrit sur `stdout`, ni ne possède de boucle de
  capture — elle prend des `cv::Mat` en entrée et rend des `LaneModel` en
  sortie, sans savoir d'où vient l'image ni ce qui en sera fait.
- **`line_detector_app`** (`src/app/`) est le harnais : analyse des arguments,
  ouverture d'une source de frames, boucle, écriture des sorties dev/test.
  C'est cette cible que l'exécutable (`src/main.cpp`) **et** la suite de
  tests linkent tous les deux — garantie qu'ils exercent exactement le même
  code, pas une variante « spéciale test ».

Trois en-têtes purement utilitaires (`VideoCaracteristics.h`, `projectTypes.h`,
`SmartAssert.h`) sont utilisés des deux côtés de cette frontière ; ils vivent
dans `src/common/`, ni dans `lib/` ni dans `app/`, pour que l'arborescence
elle-même rende visible qu'ils n'appartiennent à aucune des deux couches.

### Orchestration (`DetectLines`)

`DetectLines` assemble les sept étapes et expose deux méthodes seulement :
`compute()` (étapes 1 à 6, renvoie un `LaneModel`) et `render()` (étape 7,
dessine un `LaneModel` déjà calculé). Elle possède ses sous-composants par
valeur (`LaneMask`, `PerspectiveView`, `SlidingWindowSearch`, `LaneQuality`,
`LaneOverlay`), tous construits à partir des mêmes `VideoCaracteristics` et
`LaneConfig` — l'ordre de déclaration compte ici : `m_overlay` est déclaré
après `m_perspective` car il détient une référence dessus, et l'ordre de
construction des membres suit l'ordre de déclaration en C++ ; `m_quality` ne
détient aucune référence de ce type, sa place dans l'ordre reflète seulement
sa place dans le pipeline.

Deux structures traversent tout le pipeline :

- **`VideoCaracteristics`** — dimensions de l'image, construites une seule
  fois depuis la première frame lue, jamais depuis les métadonnées de
  `cv::VideoCapture` (souvent absentes ou fausses selon la source). Propagée
  par valeur const partout : c'est la source unique de vérité sur la
  géométrie.
- **`LaneConfig`** — tous les réglages numériques du pipeline (seuils,
  ratios de calibration BEV, paramètres des fenêtres glissantes). Centraliser
  ces valeurs ici plutôt que de les éparpiller en constantes locales, c'est ce
  qui rend le fichier consultable comme un unique point de réglage.

### La boucle applicative

`main.cpp` ne fait qu'assembler : il parse les arguments (`parse_arguments`),
construit une `FrameSource` adaptée au mode demandé, lit la première frame
(qui définit `VideoCaracteristics`), construit `DetectLines` et les
observateurs, puis délègue tout le reste à `PipelineRunner::run()`.

- **`FrameSource`** (interface à une seule méthode, `read`) a deux
  implémentations, marquées `final` : `ImageFrameSource` (une image, rendue
  une fois — le mode image fixe est un cas dégénéré du mode flux, pas un
  chemin de code à part ; fabrique `from_file`) et `VideoFrameSource`
  (enveloppe `cv::VideoCapture`, fabriques `from_file` / `from_camera` /
  `from_gstreamer`). La sous-commande choisit la classe, l'option de source
  choisit la fabrique. Les fabriques renvoient `nullptr` si la source ne peut
  pas être ouverte : un objet reçu est toujours utilisable.
- **`FrameObserver`** (interface `on_frame(index, model, image_annotée,
  compute_ms, render_ms)`) a trois implémentations : `LaneModelLogger` (une
  ligne CSV par frame), `ResultImageWriter` (mode image, écrit via un
  `ImageSink`) et `AnnotatedVideoWriter` (modes flux, `cv::VideoWriter` ouvert
  paresseusement à la réception de la première frame réelle, pour connaître sa
  taille exacte). Chaque observateur déclare via `needs_annotated_frame()`
  s'il exploite l'image rendue ; `LaneModelLogger` répond `false`, les deux
  autres répondent `true` (valeur par défaut de l'interface).
- **`PipelineRunner`** calcule une seule fois, à la construction, si **au
  moins un** observateur a besoin de l'image annotée. Si aucun n'en a besoin
  (typiquement : logger seul, sans `--record`), `DetectLines::render()` n'est
  jamais appelé et l'image transmise aux observateurs est un `cv::Mat` vide —
  un observateur qui répond `false` à `needs_annotated_frame()` ne doit jamais
  le déréférencer. La boucle s'arrête proprement à la fin du flux, sur
  `SIGINT`, ou dès qu'un observateur signale `has_fatal_error()` (utile pour
  détecter un `out/` devenu inutilisable frame par frame plutôt qu'après coup).

### Deux contrats d'échec, jamais confondus

Le code distingue strictement trois catégories d'échec possibles, chacune
avec son mécanisme propre :

- **Violation d'invariant interne** (un `cv::Mat` vide là où ça ne devrait pas
  arriver, une homographie dégénérée, un offset non fini...) → `SMART_ASSERT`.
  Toujours actif, y compris en release (pas désactivé par `NDEBUG` comme un
  `assert` standard) : sur un véhicule, ces invariants doivent rester
  vérifiés même hors debug. Un échec ici est un bug à corriger, jamais une
  situation à absorber — le programme s'arrête (`abort()`).
- **Aléa de la route** (pas de ligne visible, trop peu de pixels, largeur de
  voie non plausible) → un drapeau (`LanePolynomial::valid`,
  `LaneModel::lane_detected`), jamais un assert. C'est une situation normale
  d'exploitation, pas un bug.
- **Aléa d'environnement** (arguments invalides, fichier introuvable, dossier
  de sortie inutilisable, échec d'écriture) → `EXIT_IF_FAILED`, réservé à
  `main.cpp`. Un message lisible par un humain sur `stderr`, puis
  `exit(EXIT_FAILURE)` propre — jamais `abort()`. `line_detector_lib` et
  `line_detector_app` ne terminent jamais le processus elles-mêmes ; seul
  `main.cpp` en a le droit.

### Écriture des images (`ImageSink`)

Le résultat final et les traces de debug passent par la même interface à une
méthode, `ImageSink::save(nom, image)`. `DiskImageSink` écrit réellement sur
disque (`cv::imwrite`) ; `NullImageSink` ne fait rien et renvoie `true` (ne
rien écrire n'est pas un échec). Le sink de debug est un `NullImageSink` par
défaut et devient un `DiskImageSink` seulement si `LINE_DETECTOR_DEBUG` est
défini — ce qui permet à des étapes coûteuses (par exemple la construction de
l'image de debug des fenêtres glissantes, qui redessine chaque pixel un par
un) d'être sautées entièrement quand `is_enabled()` répond `false`, plutôt que
d'être calculées puis jetées.

## Structure du dépôt

| Chemin | Rôle |
|---|---|
| `src/main.cpp` | assemblage : arguments, source, détecteur, observateurs, boucle |
| `src/lib/DetectLines/` | orchestration du pipeline (`compute` / `render`) |
| `src/lib/LaneMask/` | étape 1 — masque binaire des marquages |
| `src/lib/PerspectiveView/` | étape 2 — vue de dessus et homographies |
| `src/lib/SlidingWindowSearch/` | étape 3 — histogramme + fenêtres glissantes |
| `src/lib/LanePolynomial/` | étape 4 — ajustement polynomial (moindres carrés) |
| `src/lib/LaneQuality/` | étape 5 — rejet des fits geometriquement incoherents |
| `src/lib/LaneGeometry/` | étape 6 — offset, courbure, reconstruction |
| `src/lib/LaneOverlay/` | étape 7 — rendu de l'overlay et du HUD |
| `src/lib/LaneTracker/` | lissage temporel du LaneModel (EMA + coasting), hors des étapes 1-7 : possédé par `PipelineRunner`, pas par `DetectLines` |
| `src/lib/LaneConfig/` | tous les réglages numériques du pipeline |
| `src/lib/LaneModel/` | structure de résultat (le signal de pilotage) |
| `src/lib/ImageSink/` | écriture d'image (résultat et debug), `Disk`/`Null` |
| `src/app/CliOptions/` | analyse des arguments de ligne de commande |
| `src/app/FrameSource/` | sources de frames : `ImageFrameSource` (fichier image), `VideoFrameSource` (fichier vidéo, caméra, GStreamer) |
| `src/app/FrameObserver/` | consommateurs par frame : log CSV, écriture image/vidéo |
| `src/app/PipelineRunner/` | boucle principale, statistiques, arrêt propre |
| `src/common/` | en-têtes partagés lib/app (types, assertions, géométrie image) |
| `tests/` | suite `doctest` — un fichier de test par composant |
| `CMakeLists.txt` | cibles `line_detector_lib`, `line_detector_app`, exécutable, tests |
| `docker/Dockerfile` | environnement de build reproductible (Debian + OpenCV) |
| `.devcontainer/` | intégration VS Code Dev Containers sur le même Dockerfile |
| `tools/make_test_image.py` | génère des images de test synthétiques dans `img_piste/` |

## Prérequis

Le programme se compile de **deux façons**, au choix (détail dans
[Installation et exécution](#installation-et-exécution)) :

- **Dans Docker** — sur une machine de développement où OpenCV n'est pas
  installé. Le conteneur (`debian:bookworm-slim` + `libopencv-dev`
  via `apt`) donne un build reproductible et fait tourner les tests. Prérequis :
  - [Docker](https://www.docker.com/products/docker-desktop/) installé et
    démarré (sur macOS : lancer Docker Desktop avant toute commande `docker`) ;
  - pour éditer avec VS Code dans le conteneur : l'extension **Dev
    Containers** (`ms-vscode-remote.remote-containers`).
- **En natif** — directement sur une machine Linux (ou macOS) où OpenCV est
  installé. C'est **la voie obligatoire sur la cible** (Raspberry Pi 5) pour
  utiliser la caméra : un exécutable compilé dans Docker ne tourne pas sur le
  Pi, car l'image Docker fournit OpenCV 4.6 et Raspberry Pi OS une autre
  version (par exemple 4.10). Prérequis : les paquets listés
  dans [Natif — Raspberry Pi 5 avec Camera Module 3](#natif--raspberry-pi-5-avec-camera-module-3).

Dans les deux cas, pour générer des images de test synthétiques : Python 3 et
Pillow (`pip install pillow`), côté hôte uniquement.

Le dépôt ne versionne pas d'images d'entrée (`img_piste/` est dans
`.gitignore` — les fichiers y sont trop lourds pour être suivis en Git). Deux
façons de s'en procurer :

```sh
python3 tools/make_test_image.py
```

génère `img_piste/img2.jpg`, `straight.jpg`, `curved.jpg`, `shifted.jpg` et
`dashed.jpg` — des routes synthétiques couvrant chacune un cas particulier
(voie droite centrée, virage, véhicule décalé, marquage discontinu pour tester
la reconstruction). On peut aussi déposer n'importe quelle photo de route dans
`img_piste/` et la passer en argument à `image --file`.

## Installation et exécution

Deux façons de compiler, qui produisent exactement le même programme — seule
la manière d'obtenir OpenCV change :

| Voie | Où | Quand l'utiliser | Sections |
|---|---|---|---|
| **Docker** | sur n'importe quelle machine avec Docker, sans OpenCV installé | développement quotidien, tests, traitement d'images et de fichiers vidéo | les trois sections « Docker — … » |
| **Natif** | sur une machine où OpenCV est installé | exécution sur la cible (Raspberry Pi 5 + caméra), ou machine de dev déjà équipée | les deux sections « Natif — … » |

Un exécutable compilé dans Docker ne s'exécute que dans Docker : pour faire
tourner le programme sur le Raspberry Pi, **compiler en natif sur le Pi**.

### Docker — en une commande depuis l'hôte

C'est le chemin le plus direct : pas besoin d'ouvrir un shell dans le
conteneur. On construit l'image une fois :

```sh
docker build -t line-detector -f docker/Dockerfile .
```

puis on compile et on exécute en montant le dépôt dans le conteneur :

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector -j \
           && mkdir -p /app/out && cd /app && /tmp/build/line_detector image --file img_piste/img2.jpg --record'
```

Le volume `-v "$(pwd):/app"` partage le dépôt avec le conteneur dans les deux
sens : modifier un fichier source ou déposer une image dans `img_piste/` ne
demande aucun rebuild de l'image Docker (`docker build` ne sert qu'à installer
les dépendances système, pas à figer le code), et `out/` apparaît directement
sur l'hôte puisqu'il vit dans le volume monté. Le dossier de build lui-même
(`/tmp/build`) reste à l'intérieur du conteneur et disparaît avec lui — rien
n'y pollue le dépôt.

Le résultat annoté atterrit dans `out/output.jpg`.

### Docker — boucle de développement interactive

Pour itérer sans relancer une commande Docker complète à chaque fois, on ouvre
un shell dans le conteneur une bonne fois, et on recompile depuis là :

```sh
docker run --rm -it -v "$(pwd):/app" -w /app line-detector bash
```

puis, à l'intérieur du conteneur :

```sh
cmake -S . -B build-linux
cmake --build build-linux --target line_detector -j
./build-linux/line_detector image --file img_piste/img2.jpg --record
```

Recompiler après une modification se limite alors à relancer la ligne
`cmake --build`, sans repasser par `docker run`.

> **Piège — cache CMake et chemin de la source.** Le chemin absolu du dossier
> source est figé dans `build-linux/CMakeCache.txt`
> (`CMAKE_HOME_DIRECTORY`). Si le projet a déjà été configuré depuis un autre
> chemin — par exemple `/app` via `docker run -v "$(pwd):/app"`, alors qu'un
> Dev Container monte le même dépôt sur `/workspaces/line-detector` — CMake
> refuse de réutiliser ce cache :
>
> ```
> CMake Error: The source "/workspaces/line-detector/CMakeLists.txt" does not match
> the source "/app/CMakeLists.txt" used to generate cache.
> ```
>
> Un build hors-source n'est pas relogeable d'un chemin à l'autre : supprimer
> le dossier de build (`rm -rf build-linux`) et reconfigurer, ou utiliser un
> dossier de build différent par contexte (`cmake -S . -B build-autre`).

### Docker — avec VS Code (Dev Containers)

VS Code peut ouvrir le projet directement à l'intérieur du même conteneur, en
réutilisant le `Dockerfile` existant (`.devcontainer/devcontainer.json` s'y
réfère, il n'y a pas de configuration dupliquée) :

1. Installer l'extension **Dev Containers**.
2. `Cmd/Ctrl + Shift + P` → **« Dev Containers: Reopen in Container »**.
3. Une fois le conteneur ouvert, utiliser un terminal intégré avec les mêmes
   commandes que la section précédente (« Docker — boucle de développement
   interactive ») — le shell est déjà dans le conteneur, il n'y a pas de
   `docker run` à taper.

Le dossier de build est réglé sur `build-linux/` par
`.devcontainer/devcontainer.json` (`cmake.buildDirectory`), délibérément
distinct d'un éventuel `build/` généré en dehors du conteneur — les deux
caches CMake, l'un pointant vers un toolchain Linux et l'autre vers celui de l'hôte, ne
doivent jamais se mélanger (cf. le piège ci-dessus). Les extensions C++ et
CMake Tools sont installées automatiquement dans le conteneur, avec
autocomplétion et configuration CMake via l'interface VS Code.

### Natif — machine avec OpenCV installé

Si OpenCV est installé au niveau système (par exemple via Homebrew sur macOS,
ou le paquet `libopencv-dev` d'une distribution Linux), le build se fait
directement, sans Docker :

```sh
cmake -S . -B build && cmake --build build -j
./build/line_detector image --file img_piste/img2.jpg --record
```

L'exécutable est lié à la version d'OpenCV de cette machine. Pour la source
`--gstreamer`, OpenCV doit avoir été compilé avec le backend GStreamer (c'est
le cas des paquets Debian et Raspberry Pi OS ; `./build/line_detector_tests`
le vérifie).

### Natif — Raspberry Pi 5 avec Camera Module 3

La voie à suivre pour exécuter le programme sur la cible avec la caméra CSI :
**compiler directement sur le Pi**. Un exécutable construit dans Docker n'y
tourne pas : l'image Docker (Debian Bookworm) fournit OpenCV 4.6, alors que
Raspberry Pi OS fournit une autre version (par exemple 4.10),
et un exécutable lié à l'une ne trouve pas les bibliothèques de l'autre.

```sh
sudo apt update
sudo apt install \
  build-essential cmake pkg-config \
  libopencv-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-libcamera \
  gstreamer1.0-tools
```

| Paquet | Rôle | Nécessaire ? |
|---|---|---|
| `build-essential` | compilateur C++ (`g++`), `make` | pour compiler |
| `cmake` | outil de build du projet (≥ 3.18) | pour compiler |
| `pkg-config` | résolution des dépendances de build | pour compiler |
| `libopencv-dev` | OpenCV (en-têtes + bibliothèques), compilé avec le backend GStreamer | oui |
| `gstreamer1.0-plugins-base` | `videoconvert` et `appsink` (fin de pipeline ajoutée par le programme), `videotestsrc` (tests) | oui |
| `gstreamer1.0-libcamera` | élément `libcamerasrc` : lit la caméra CSI via libcamera ; installe la version de libcamera qui lui correspond | pour la caméra |
| `gstreamer1.0-tools` | `gst-launch-1.0`, `gst-inspect-1.0`, pour diagnostiquer un pipeline | recommandé |

Déjà présents sur Raspberry Pi OS, sans rien installer : libcamera et ses
modules de réglage pour l'ISP du Pi 5 (dont le fichier de tuning de l'imx708),
et `rpicam-apps` (`rpicam-hello`), utile seulement pour vérifier que la caméra
est vue. L'utilisateur doit appartenir au groupe `video` (c'est le cas de
l'utilisateur par défaut). Docker et `libcamera-v4l2` (`libcamerify`) ne sont
pas nécessaires.

Compiler, vérifier l'installation, puis lancer sur la caméra :

```sh
cmake -S . -B build && cmake --build build -j
./build/line_detector_tests
rpicam-hello --list-cameras
./build/line_detector video --gstreamer "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12"
```

`line_detector_tests` vérifie aussi l'installation : s'il manque le backend
GStreamer d'OpenCV ou `gstreamer1.0-plugins-base`, les tests de
`from_gstreamer` échouent avec un message qui le dit. `rpicam-hello
--list-cameras` doit lister l'imx708. Le `format=NV12` du pipeline est
indispensable (cf. [Source GStreamer](#source-gstreamer)).

Validé sur Raspberry Pi 5 + Camera Module 3 (imx708), libcamera 0.7.2.

## Utilisation du programme

```
line_detector image --file <chemin> [--record]
line_detector video (--file <chemin> | --camera [index] | --gstreamer <pipeline source>) [--record]
```

La ligne de commande a deux axes indépendants :

- la **sous-commande** (`image` ou `video`, obligatoire, en première
  position) dit quel résultat on veut : une frame, ou un flux ;
- l'**option de source** (exactement une, obligatoire) dit d'où viennent les
  pixels.

| Commande | Effet | Sortie avec `--record` |
|---|---|---|
| `image --file <chemin>` | traite une image fixe (une seule frame) | `out/output.jpg` |
| `video --file <chemin>` | traite un fichier vidéo | `out/output.avi` |
| `video --camera [index]` | traite une caméra V4L2, typiquement une webcam USB (index par défaut : `0`) | `out/output.avi` |
| `video --gstreamer <pipeline source>` | traite un flux GStreamer (caméra CSI, RTSP, …) | `out/output.avi` |

`image` n'accepte que `--file` pour l'instant : `image --camera` et
`image --gstreamer` sont rejetés avec un message explicite. Sans sous-commande
ou sans source, le programme affiche l'aide et sort en erreur — il n'y a pas
de valeur par défaut.

### Source GStreamer

`--gstreamer` attend **seulement la partie source** du pipeline. Le programme
ajoute lui-même la fin :

```
 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1
```

Le détecteur exige du BGR, et `drop=true max-buffers=1` garde toujours la
frame la plus récente : si le traitement est plus lent que la caméra, les
frames en retard sont jetées au lieu de s'accumuler. Une valeur contenant un
élément puits (`appsink`, `fakesink`… : pipeline complet collé par habitude)
ou un élément vide (`!` final ou doublé) est rejetée, car OpenCV resterait
bloqué à l'ouverture au lieu d'échouer.

Exemple, Camera Module 3 (imx708) sur Raspberry Pi 5 — validé avec
libcamera 0.7.2. Sur le Pi 5, la caméra CSI passe par la pile libcamera :
`/dev/video0` y est un nœud Bayer brut, inutilisable avec `--camera`.

```sh
sudo apt install gstreamer1.0-libcamera
line_detector video --gstreamer "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12"
```

**Le `format=NV12` est indispensable.** Sans format, `libcamerasrc` se rabat
sur le flux Bayer brut du capteur (`configuring streams: (0) 1536x864-SBGGR16/RAW`
dans la trace libcamera), que `videoconvert` ne sait pas convertir : la
négociation échoue (`streaming stopped, reason not-negotiated`) et le
programme sort avec « Impossible d'ouvrir la source demandee. ». Avec
`format=NV12`, l'ISP du Pi produit directement l'image à la taille demandée.

Prérequis : OpenCV compilé avec le backend GStreamer (c'est le cas des paquets
Debian) et `gstreamer1.0-plugins-base`.

**Limite connue** : si le **premier** élément du pipeline n'existe pas (faute
de frappe, plugin non installé), OpenCV (constaté avec la version 4.6) reste bloqué à
l'ouverture, sans message ; `Ctrl-C` termine le programme. Vérifier d'abord
la source hors programme (`gst-launch-1.0` est fourni par le paquet
`gstreamer1.0-tools`, à installer avec `sudo apt install gstreamer1.0-tools`) :

```sh
gst-launch-1.0 -v libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12 ! videoconvert ! fakesink
```

Le pipeline doit tourner jusqu'au `Ctrl-C`, et `-v` affiche les formats
négociés. Pour diagnostiquer un échec, relancer avec `GST_DEBUG=2` devant la
commande. Cette ligne se termine par `fakesink` : ne pas la coller telle
quelle dans `--gstreamer`, qui la refuserait. N'y reprendre que la partie
avant `! videoconvert`.

### Écriture du résultat et arrêt

**`--record` gouverne l'écriture du résultat, uniformément pour les deux
sous-commandes** — le mode image fixe n'est pas un cas à part, c'est un cas dégénéré du
mode flux (une seule frame, puis fin). Sans `--record`, le programme n'écrit
**aucun fichier** et n'exécute **aucun rendu** : aucun observateur ne
réclamant l'image annotée, l'étape overlay est purement et simplement sautée
(`render_ms` reste à `0` dans le CSV). C'est le mode à utiliser pour consommer
le signal de pilotage au coût de calcul minimal.

`Ctrl-C` arrête proprement la boucle et referme le fichier vidéo en cours
d'écriture ; un second `Ctrl-C` termine toujours le processus, y compris si la
lecture caméra est bloquée sur un appel qui ne reviendra jamais tester le
drapeau d'arrêt.

### `stdout` et `stderr` sont strictement séparés

`stdout` ne porte **que** le signal de pilotage : une ligne d'en-tête suivie
d'une ligne CSV par frame (séparateur `;`, format à virgule fixe — jamais de
notation scientifique, pour que les colonnes restent stables même sur un grand
rayon de courbure) :

```
frame_index;lane_detected;normalized_offset;lateral_offset_px;curvature_radius_px;reconstructed;coasted;compute_ms;render_ms
```

`stderr` porte tout ce qui s'adresse à un humain : messages d'erreur, résumé
final (frames traitées / détections / reconstructions / durée moyenne / FPS,
part de rendu détaillée entre parenthèses), et le chemin du fichier écrit si
`--record` est passé. Rediriger `stdout` seul suffit donc à consommer le
signal sans avoir à filtrer du texte destiné à un humain :

```sh
./build-linux/line_detector video --camera 0 > pilotage.csv
```

Rappel de convention : un `normalized_offset` négatif signifie que le véhicule
est décalé **à gauche**. `reconstructed` vaut `1` quand un seul marquage était
visible et que l'autre a été reconstruit par décalage — le signal reste
exploitable mais dégradé. `coasted` vaut `1` quand cette frame n'a pas de
détection fraîche et que `LaneTracker` reconduit le dernier modèle lissé
connu — signal figé, à distinguer d'une détection fraîche même si
`lane_detected` reste à `1` dans les deux cas.

### Dossier de sortie

Par défaut `out/`, surchargeable par la variable d'environnement
`LINE_DETECTOR_OUT`. Le programme crée ce dossier lui-même s'il n'existe pas
(`cv::imwrite` et `cv::VideoWriter` ne le font pas), mais uniquement s'il a
effectivement quelque chose à y écrire — `--record`, ou `LINE_DETECTOR_DEBUG`.
Sans l'un ou l'autre, aucun dossier n'est créé ni même testé : un `out/`
inaccessible n'est alors pas une raison d'échec, ce qui compte au premier
lancement sur une caméra sans `--record`, avant même d'avoir pensé à créer ce
dossier.

### Erreurs et code de sortie

Une erreur d'exécution (arguments invalides, source introuvable, dossier de
sortie inutilisable, échec d'écriture) affiche un message lisible sur `stderr`
et termine proprement le programme avec `EXIT_FAILURE` (1) — jamais de
plantage : c'est un aléa d'environnement attendu, pas un bug interne (cf.
[Deux contrats d'échec, jamais confondus](#deux-contrats-déchec-jamais-confondus)
ci-dessus).

### Cadence de la vidéo de sortie

La vidéo annotée est toujours écrite à une cadence fixe de 30 fps, quelle que
soit la cadence réelle de la source. Une vidéo issue d'une source plus lente
ou plus rapide que 30 fps paraîtra donc accélérée ou ralentie à la relecture
— ce n'est pas un bug, c'est un choix pour ne pas dépendre d'un `fps()` que
`cv::VideoCapture` n'expose pas toujours de façon fiable selon la source.

## Traces de debug

Exécuter avec la variable d'environnement `LINE_DETECTOR_DEBUG` non vide écrit
les étapes intermédiaires du pipeline dans le dossier de sortie :

```sh
LINE_DETECTOR_DEBUG=1 ./build-linux/line_detector image --file img_piste/img2.jpg --record
```

```sh
LINE_DETECTOR_DEBUG=1 LINE_DETECTOR_OUT="/tmp/line_detector_out" ./build-linux/line_detector image --file img_piste/img2.jpg --record

LINE_DETECTOR_DEBUG=1 LINE_DETECTOR_OUT="data/out" ./build-linux/line_detector image --file data/img_piste/test.jpg --record
```

| Fichier | Contenu | Condition |
|---|---|---|
| `out/debug_01_mask.jpg` | masque binaire des marquages (étape 1) | `LINE_DETECTOR_DEBUG` |
| `out/debug_02_bev.jpg` | masque en vue de dessus (étape 2) | `LINE_DETECTOR_DEBUG` |
| `out/debug_02a_trapeze.jpg` | trapèze source superposé à l'image couleur | `LINE_DETECTOR_DEBUG` |
| `out/debug_02b_bev_color.jpg` | vue de dessus en couleur (pas seulement le masque) | `LINE_DETECTOR_DEBUG` |
| `out/debug_03_windows.jpg` | fenêtres glissantes, pixels gauche en rouge / droite en bleu | `LINE_DETECTOR_DEBUG` |
| `out/debug_04_fit.jpg` | polynômes ajustés tracés sur la vue de dessus | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` ne rejette aucun côté |
| `out/debug_04b_quality.jpg` | fits d'origine + raison du rejet (largeur incohérente ou pixels insuffisants) | `LINE_DETECTOR_DEBUG`, **si** `LaneQuality` rejette au moins un côté |
| `out/debug_04c_tracker.jpg` | polynômes bruts vs lissés par `LaneTracker` (rouge/bleu = brut, vert/jaune = lissé) | `LINE_DETECTOR_DEBUG` |
| `out/debug_05_overlay.jpg` | overlay final, identique à `out/output.jpg` | `LINE_DETECTOR_DEBUG` **et** `--record` |

`debug_05_overlay.jpg` est le seul qui demande en plus `--record` : il vient
de l'étape de rendu (étape 7), qui n'est exécutée que si un observateur la
réclame. `debug_04_fit.jpg` et `debug_04b_quality.jpg` sont mutuellement
exclusifs — le premier suppose que `LaneQuality` (étape 5) n'a rien démoté, le
second n'existe que si elle l'a fait. Toutes les autres traces viennent de
`compute()`, exécuté dans tous les cas.

En mode flux (vidéo ou caméra), les noms de fichiers sont fixes : chaque frame
écrase la précédente, seule la dernière subsiste. C'est voulu — ces traces
servent à caler la calibration en direct, pas à archiver une séquence entière.

**Calibrer la vue de dessus.** Les quatre ratios `src_top_width_ratio` /
`src_top_y_ratio` / `src_bottom_width_ratio` / `src_bottom_y_ratio` de
`LaneConfig.h` définissent le trapèze source (cf. [étape
2](#2-redresser-en-vue-de-dessus-perspectiveview)). Les valeurs par défaut
actuelles (`src_bottom_y_ratio = 0.30`, `src_bottom_width_ratio = 0.44`) sont
calibrées à la main pour la caméra du véhicule, montée très bas (quelques
centimètres du sol) — un montage bas fait sortir les marquages du cadre par
les côtés avant d'atteindre le bas de l'image, d'où un bord bas du trapèze
remonté et resserré plutôt que collé aux coins de l'image (qui serait le
réglage adapté à une caméra montée haute, `src_bottom_y_ratio` proche de `1.0`
et `src_bottom_width_ratio` proche de `0.5`). Ces quatre valeurs sont des
constantes de compilation alors que le réglage est propre à chaque
caméra/montage — un mécanisme de calibration sans recompilation (variable
d'environnement, fichier de config, flag CLI) reste à faire. En attendant, le
réglage se fait en modifiant `LaneConfig.h` et en inspectant
`out/debug_02a_trapeze.jpg` (le trapèze superposé à l'image d'origine, pour
vérifier qu'il englobe bien les deux marquages) puis `out/debug_02_bev.jpg`,
jusqu'à ce que des lignes droites parallèles y apparaissent verticales et
parallèles.

## Tests

Suite de tests unitaires `doctest` (header unique vendored dans
`tests/doctest.h`, pas de dépendance externe à installer), cible CMake séparée
`line_detector_tests`. Elle se lance dans les deux voies de compilation.

En natif (par exemple sur le Raspberry Pi), ou dans un shell ouvert dans le
conteneur :

```sh
cmake -S . -B build-linux
cmake --build build-linux --target line_detector_tests -j
./build-linux/line_detector_tests
```

Dans Docker, depuis l'hôte en une commande, sur le même modèle que la
compilation :

```sh
docker run --rm -v "$(pwd):/app" -w /app line-detector \
  bash -c 'cmake -S /app -B /tmp/build && cmake --build /tmp/build --target line_detector_tests -j \
           && /tmp/build/line_detector_tests'
```

Les fichiers `tests/test_*.cpp` sont ramassés par un `file(GLOB ...)` dans
`CMakeLists.txt` : ajouter un nouveau fichier de test est pris en compte
automatiquement au prochain `cmake -S -B` (la reconfiguration est nécessaire
puisque la liste de fichiers glob est évaluée à la configuration, pas au
build).

Chaque composant du pipeline (`LaneMask`, `PerspectiveView`,
`SlidingWindowSearch`, `LanePolynomial`, `LaneGeometry`, `LaneOverlay`) et
chaque composant applicatif (`CliOptions`, `FrameSource`, `FrameObserver`,
`AnnotatedVideoWriter`, `PipelineRunner`) a son propre fichier de test. Trois
tests vont plus loin, bout en bout :

- deux dans `tests/test_integration.cpp`, qui exercent `DetectLines` en entier
  sur des images synthétiques — dont une voie courbe, pour vérifier que le fit
  polynomial n'est pas aplati par erreur en une droite ;
- un dans `tests/test_video_integration.cpp`, pendant du mode flux : une voie
  qui dérive progressivement, passée dans un `PipelineRunner` complet, qui
  vérifie que `normalized_offset` évolue de façon monotone d'une frame à
  l'autre — la preuve que le mode vidéo suit réellement quelque chose, pas
  seulement qu'il ne plante pas.

Il n'y a pas de linter ni de CI configurés sur ce dépôt à ce jour.

## Limites connues

Le mode vidéo (fichier et caméra) lisse désormais le `LaneModel` à travers les
frames (`LaneTracker`), mais il n'y a toujours pas de recherche localisée
autour du fit de la frame précédente (qui accélérerait le traitement en
évitant de relancer une recherche par histogramme à chaque frame). Restent
également au programme la correction de distorsion caméra (le pipeline
suppose une caméra sans distorsion notable) et le passage d'un signal en
pixels à un signal en unités métriques, qui suppose une calibration
caméra/sol supplémentaire.

commandes :
LINE_DETECTOR_DEBUG=1 ./line_detector video --gstreamer "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12" --record

LINE_DETECTOR_DEBUG=1 ./line_detector video --gstreamer "libcamerasrc ! video/x-raw,width=2304,height=1296,format=NV12 ! videoscale ! video/x-raw,width=1280,height=720" --record

LINE_DETECTOR_DEBUG=1 ./line_detector video --gstreamer "libcamerasrc ! video/x-raw,width=2304,height=1296,format=NV12" --record
