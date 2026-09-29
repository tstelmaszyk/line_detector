/// @file
/// @brief Implémentation de l'analyse des arguments de ligne de commande.

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <string>

#include "CliOptions/CliOptions.h"

const ::std::string USAGE_MESSAGE =
  "Usage : line_detector image --file <chemin> [--record]\n"
  "        line_detector video (--file <chemin> | --camera [index] | --gstreamer <pipeline source>) [--record]";  ///< Aide.

namespace
{

const ::std::string COMMAND_IMAGE = "image";          ///< Sous-commande : une frame, résultat image.
const ::std::string COMMAND_VIDEO = "video";          ///< Sous-commande : un flux, résultat vidéo.
const ::std::string FLAG_FILE = "--file";             ///< Source : fichier.
const ::std::string FLAG_CAMERA = "--camera";         ///< Source : caméra.
const ::std::string FLAG_GSTREAMER = "--gstreamer";   ///< Source : partie source d'un pipeline GStreamer.
const ::std::string FLAG_RECORD = "--record";         ///< Flag d'écriture du résultat.
const ::std::string GSTREAMER_SINK_ELEMENT = "appsink";  ///< Ajouté par VideoFrameSource : interdit dans la valeur.

const ::std::string ERROR_MISSING_COMMAND = "Sous-commande manquante : image ou video";  ///< argv[1] absent ou flag.
const ::std::string ERROR_UNKNOWN_COMMAND = "Sous-commande inconnue : ";                ///< argv[1] non reconnu.
const ::std::string ERROR_MISSING_SOURCE =
  "Source manquante : une parmi --file, --camera, --gstreamer";  ///< Aucune source.
const ::std::string ERROR_CONFLICTING_SOURCES =
  "Sources exclusives : une seule parmi --file, --camera, --gstreamer";  ///< Deux sources.
const ::std::string ERROR_NOT_WITH_IMAGE = " n'est pas disponible avec image";  ///< Source non supportée par image.
const ::std::string ERROR_MISSING_VALUE = "Valeur manquante apres : ";          ///< Flag sans valeur exploitable.
const ::std::string ERROR_GSTREAMER_SINK =
  "--gstreamer attend seulement la partie source du pipeline (sans appsink) : "
  "la fin est ajoutee par le programme";  ///< Pipeline complet collé par habitude.
const ::std::string ERROR_UNKNOWN_FLAG = "Flag inconnu : ";                     ///< Flag non reconnu.
const ::std::string ERROR_POSITIONAL = "Argument positionnel non supporte : ";  ///< argv nu en trop.
const ::std::string ERROR_CAMERA_INDEX = "Index camera invalide : ";            ///< Index non numérique.

const int COMMAND_ARGUMENT_INDEX = 1;  ///< Position de la sous-commande (argv[0] = nom du programme).
const int FIRST_OPTION_INDEX = 2;      ///< Premier argument après la sous-commande.
const char FLAG_PREFIX = '-';          ///< Préfixe d'un flag.
const int DECIMAL_BASE = 10;           ///< Base de conversion de l'index caméra.

/// @brief Indique si un argument a la forme d'un flag.
/// @param p_argument Argument à tester.
/// @return true si l'argument commence par un tiret.
bool looks_like_flag( const ::std::string& p_argument )
  {
  const bool is_empty = p_argument.empty();

  if ( is_empty )
    {
    return false;
    }

  const bool starts_with_dash = ( FLAG_PREFIX == p_argument[0] );
  return starts_with_dash;
  }

/// @brief Indique si une chaîne est vide ou ne contient que des espaces.
/// @param p_text Chaîne à tester.
/// @return true si aucun caractère visible n'est présent.
bool is_blank( const ::std::string& p_text )
  {
  for ( const char character : p_text )
    {
    const bool is_space = ( 0 != ::std::isspace( static_cast< unsigned char >( character ) ) );

    if ( !is_space )
      {
      return false;
      }
    }

  return true;
  }

/// @brief Lit la valeur obligatoire qui suit un flag.
/// @param p_argument_count Nombre d'arguments.
/// @param p_arguments      Tableau d'arguments.
/// @param p_flag_index     Index du flag dans p_arguments.
/// @param p_value          Valeur lue en sortie (inchangée en cas d'échec).
/// @return true si une valeur exploitable suit le flag : présente, pas un flag, pas blanche.
bool read_required_value( int p_argument_count, char** p_arguments, int p_flag_index, ::std::string& p_value )
  {
  const int value_index = p_flag_index + 1;
  const bool value_is_missing = ( value_index >= p_argument_count );

  if ( value_is_missing )
    {
    return false;
    }

  const ::std::string candidate_value = p_arguments[value_index];
  const bool value_looks_like_flag = looks_like_flag( candidate_value );
  const bool value_is_blank = is_blank( candidate_value );

  if ( value_looks_like_flag || value_is_blank )
    {
    return false;
    }

  p_value = candidate_value;
  return true;
  }

/// @brief Convertit un index caméra écrit en décimal.
/// @param p_text  Texte à convertir.
/// @param p_index Index converti en sortie (inchangé en cas d'échec).
/// @return true si p_text est un entier décimal complet qui tient dans un int.
bool parse_camera_index( const ::std::string& p_text, CameraIndex& p_index )
  {
  errno = 0;
  char* conversion_end = nullptr;
  const long converted_index = ::std::strtol( p_text.c_str(), &conversion_end, DECIMAL_BASE );
  const bool conversion_ok =
    ( ( nullptr != conversion_end ) && ( '\0' == *conversion_end ) && !p_text.empty() );
  const bool has_range_error = ( ERANGE == errno );
  const bool exceeds_int_max = ( INT_MAX < converted_index );
  const bool exceeds_int_min = ( INT_MIN > converted_index );

  if ( !conversion_ok || has_range_error || exceeds_int_max || exceeds_int_min )
    {
    return false;
    }

  p_index = static_cast< CameraIndex >( converted_index );
  return true;
  }

} // namespace

int parse_arguments( int p_argument_count, char** p_arguments, CliOptions& p_options )
{
  // 1. Sous-commande : obligatoire, en première position.
  const bool command_is_missing = ( COMMAND_ARGUMENT_INDEX >= p_argument_count );

  if ( command_is_missing )
    {
    p_options.error_message = ERROR_MISSING_COMMAND;
    return EXIT_FAILURE;
    }

  const ::std::string command = p_arguments[COMMAND_ARGUMENT_INDEX];
  const bool is_image_command = ( COMMAND_IMAGE == command );
  const bool is_video_command = ( COMMAND_VIDEO == command );
  const bool command_looks_like_flag = looks_like_flag( command );

  if ( is_image_command )
    {
    p_options.media_kind = MEDIA_KIND_IMAGE;
    }
  else if ( is_video_command )
    {
    p_options.media_kind = MEDIA_KIND_VIDEO;
    }
  else if ( command_looks_like_flag )
    {
    p_options.error_message = ERROR_MISSING_COMMAND;
    return EXIT_FAILURE;
    }
  else
    {
    p_options.error_message = ERROR_UNKNOWN_COMMAND + command;
    return EXIT_FAILURE;
    }

  // 2. Options : exactement une source, et --record.
  bool source_already_set = false;
  int argument_index = FIRST_OPTION_INDEX;

  while ( argument_index < p_argument_count )
    {
    const ::std::string argument = p_arguments[argument_index];
    const bool is_file_flag = ( FLAG_FILE == argument );
    const bool is_camera_flag = ( FLAG_CAMERA == argument );
    const bool is_gstreamer_flag = ( FLAG_GSTREAMER == argument );
    const bool is_source_flag = ( is_file_flag || is_camera_flag || is_gstreamer_flag );

    if ( is_source_flag && source_already_set )
      {
      p_options.error_message = ERROR_CONFLICTING_SOURCES;
      return EXIT_FAILURE;
      }

    // image ne lit qu'un fichier : les autres sources sont prévues par la
    // grammaire mais pas implémentées.
    const bool source_needs_video = ( is_camera_flag || is_gstreamer_flag );

    if ( is_image_command && source_needs_video )
      {
      p_options.error_message = argument + ERROR_NOT_WITH_IMAGE;
      return EXIT_FAILURE;
      }

    if ( is_file_flag )
      {
      const bool value_ok = read_required_value( p_argument_count, p_arguments, argument_index, p_options.input_path );

      if ( !value_ok )
        {
        p_options.error_message = ERROR_MISSING_VALUE + argument;
        return EXIT_FAILURE;
        }

      p_options.source_kind = SOURCE_KIND_FILE;
      source_already_set = true;
      const int value_index = argument_index + 1;
      argument_index = value_index + 1;
      }
    else if ( is_gstreamer_flag )
      {
      const bool value_ok =
        read_required_value( p_argument_count, p_arguments, argument_index, p_options.gstreamer_pipeline );

      if ( !value_ok )
        {
        p_options.error_message = ERROR_MISSING_VALUE + argument;
        return EXIT_FAILURE;
        }

      // Un pipeline complet (collé par habitude) bloquerait l'ouverture : la
      // fin est ajoutée par VideoFrameSource::from_gstreamer.
      const bool contains_sink = ( ::std::string::npos != p_options.gstreamer_pipeline.find( GSTREAMER_SINK_ELEMENT ) );

      if ( contains_sink )
        {
        p_options.error_message = ERROR_GSTREAMER_SINK;
        return EXIT_FAILURE;
        }

      p_options.source_kind = SOURCE_KIND_GSTREAMER;
      source_already_set = true;
      const int value_index = argument_index + 1;
      argument_index = value_index + 1;
      }
    else if ( is_camera_flag )
      {
      // Valeur optionnelle : l'index suit le flag, ou vaut le défaut.
      p_options.source_kind = SOURCE_KIND_CAMERA;
      p_options.camera_index = DEFAULT_CAMERA_INDEX;
      source_already_set = true;
      argument_index = argument_index + 1;

      const bool has_next_argument = ( argument_index < p_argument_count );

      if ( has_next_argument )
        {
        const ::std::string next_argument = p_arguments[argument_index];
        const bool next_is_flag = looks_like_flag( next_argument );

        if ( !next_is_flag )
          {
          const bool index_ok = parse_camera_index( next_argument, p_options.camera_index );

          if ( !index_ok )
            {
            p_options.error_message = ERROR_CAMERA_INDEX + next_argument;
            return EXIT_FAILURE;
            }

          argument_index = argument_index + 1;
          }
        }
      }
    else if ( FLAG_RECORD == argument )
      {
      p_options.record = true;
      argument_index = argument_index + 1;
      }
    else
      {
      const bool is_flag = looks_like_flag( argument );
      p_options.error_message = is_flag ? ( ERROR_UNKNOWN_FLAG + argument )
                                        : ( ERROR_POSITIONAL + argument );
      return EXIT_FAILURE;
      }
    }

  if ( !source_already_set )
    {
    p_options.error_message = ERROR_MISSING_SOURCE;
    return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
