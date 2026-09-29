# =====================================================================
# 1. IMAGE DE BASE
# ---------------------------------------------------------------------
# Un conteneur part toujours d'une "image de base" : un système Linux
# minimal. On choisit Debian 12 ("bookworm") car ses dépôts contiennent
# OpenCV, la dépendance du projet. Le conteneur sert surtout à obtenir un
# build reproductible, identique à la cible Raspberry Pi.
# Le suffixe "-slim" = version allégée (moins de paquets inutiles).
# =====================================================================
FROM debian:bookworm-slim

# =====================================================================
# 2. INSTALLATION DES DÉPENDANCES
# ---------------------------------------------------------------------
# RUN exécute une commande PENDANT la construction de l'image.
# On installe :
#   - build-essential : le compilateur C++ (g++), make, etc.
#   - cmake           : l'outil de build du projet
#   - pkg-config      : utilitaire de résolution de dépendances de build
#   - libopencv-dev   : OpenCV (en-têtes + libs) -> find_package(OpenCV)
#   - gstreamer1.0-plugins-base : videotestsrc, videoconvert, appsink, pour
#                     les tests de VideoFrameSource::from_gstreamer (deja tire
#                     par libopencv-dev aujourd'hui ; explicite pour ne pas en
#                     dependre par accident)
#
# Astuce : on enchaîne update + install + nettoyage dans UN SEUL RUN.
# Chaque RUN crée une "couche" (layer) dans l'image ; en regroupant et
# en supprimant le cache apt à la fin, l'image finale reste plus légère.
# =====================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        libopencv-dev \
        gstreamer1.0-plugins-base \
    && rm -rf /var/lib/apt/lists/*

# =====================================================================
# 3. RÉPERTOIRE DE TRAVAIL
# ---------------------------------------------------------------------
# WORKDIR = le dossier "courant" à l'intérieur du conteneur.
# Toutes les commandes suivantes s'exécuteront depuis /app.
# (Il est créé automatiquement s'il n'existe pas.)
# =====================================================================
WORKDIR /app

# =====================================================================
# 4. COPIE DU CODE SOURCE
# ---------------------------------------------------------------------
# COPY <source_sur_le_Mac> <destination_dans_le_conteneur>
# Le "." veut dire "tout le dossier courant". Grâce au .dockerignore,
# on NE copie PAS le dossier build/ ni le .git/ (voir ce fichier).
# =====================================================================
COPY . /app

# =====================================================================
# 5. COMPILATION
# ---------------------------------------------------------------------
# On reproduit exactement les étapes décrites dans le CLAUDE.md :
#   build hors-source, cmake puis make.
# À la fin, l'exécutable s'appelle "line_detector" et se trouve dans /app/build.
# =====================================================================
RUN mkdir -p build && cd build && cmake .. && make

# Le dossier de sortie est code en dur (out/) cote application et cv::imwrite ne
# le cree pas : on le pre-cree ici pour que le CMD par defaut fonctionne sans
# montage de volume. En usage reel, monter -v "<hote>/out:/app/out" pour
# recuperer le resultat sur l'hote.
RUN mkdir -p out

# =====================================================================
# 6. COMMANDE PAR DÉFAUT
# ---------------------------------------------------------------------
# CMD = ce qui s'exécute quand on lance le conteneur (docker run).
# Ici on lance simplement l'exécutable compilé.
# =====================================================================
CMD ["./build/line_detector", "--image", "img_piste/img2.jpg", "--record"]
