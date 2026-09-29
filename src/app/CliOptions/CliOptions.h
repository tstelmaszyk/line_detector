#pragma once

/// @file
/// @brief Options de ligne de commande et analyse des arguments.

#include <string>

#include "projectTypes.h"

/// @brief Nature du résultat demandé (sous-commande, premier argument).
enum e_media_kind
  {
  MEDIA_KIND_IMAGE = 0,  ///< Une frame, résultat output.jpg.
  MEDIA_KIND_VIDEO,      ///< Un flux, résultat output.avi.
  MEDIA_KIND_COUNT       ///< Nombre d'éléments.
  };

/// @brief Provenance des pixels (option de source).
enum e_source_kind
  {
  SOURCE_KIND_FILE = 0,   ///< --file <chemin>
  SOURCE_KIND_CAMERA,     ///< --camera [index]
  SOURCE_KIND_GSTREAMER,  ///< --gstreamer <pipeline source>
  SOURCE_KIND_COUNT       ///< Nombre d'éléments.
  };

/// @brief Message d'aide affiché sur stderr en cas d'arguments invalides.
extern const ::std::string USAGE_MESSAGE;

/// @brief Index de caméra utilisé quand --camera est donné sans valeur.
static const CameraIndex DEFAULT_CAMERA_INDEX = 0;

/// @brief Options issues de la ligne de commande.
///
/// Les valeurs par défaut de media_kind et source_kind servent seulement à
/// initialiser la structure : parse_arguments exige une sous-commande et une
/// source, et échoue sinon.
struct CliOptions
  {
  e_media_kind media_kind = MEDIA_KIND_IMAGE;       ///< Sous-commande demandée.
  e_source_kind source_kind = SOURCE_KIND_FILE;     ///< Source demandée.
  ::std::string input_path;                         ///< Chemin (--file).
  CameraIndex camera_index = DEFAULT_CAMERA_INDEX;  ///< Index caméra (--camera).
  ::std::string gstreamer_pipeline;                 ///< Partie source du pipeline (--gstreamer).
  bool record = false;                              ///< true si --record est passé : écrit le résultat sur disque.
  ::std::string error_message;                      ///< Vide si les arguments sont valides.
  };

/// @brief Analyse les arguments de la ligne de commande.
///
/// Grammaire : `image --file <chemin> [--record]` ou
/// `video (--file <chemin> | --camera [index] | --gstreamer <pipeline source>) [--record]`.
/// La sous-commande est obligatoire et en première position ; exactement une
/// source est obligatoire. `image` n'accepte que --file.
/// @param p_argument_count Nombre d'arguments (argv[0] inclus).
/// @param p_arguments      Tableau d'arguments.
/// @param p_options        Options remplies en sortie (error_message en cas d'échec).
/// @return EXIT_SUCCESS si les arguments sont valides, EXIT_FAILURE sinon.
int parse_arguments( int p_argument_count, char** p_arguments, CliOptions& p_options );
