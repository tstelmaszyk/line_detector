#include "doctest.h"

#include <cstdlib>
#include <string>
#include <vector>

#include "CliOptions/CliOptions.h"

namespace
{

/// @brief Construit un tableau argv modifiable a partir d'une liste de chaines.
class ArgumentVector
  {
  public:
    /// @brief Copie les arguments et prepare les pointeurs argv.
    /// @param p_arguments Arguments, argv[0] inclus.
    explicit ArgumentVector( const ::std::vector< ::std::string >& p_arguments )
      : m_storage( p_arguments ),
        m_pointers()
      {
      m_pointers.reserve( m_storage.size() );

      for ( ::std::string& argument : m_storage )
        {
        m_pointers.push_back( &argument[0] );
        }
      }

    /// @brief Nombre d'arguments.
    int count() const
      {
      const int argument_count = static_cast< int >( m_pointers.size() );
      return argument_count;
      }

    /// @brief Tableau argv.
    char** values()
      {
      return m_pointers.data();
      }

  private:
    ::std::vector< ::std::string > m_storage;  ///< Copies des arguments.
    ::std::vector< char* > m_pointers;         ///< Pointeurs vers les copies.
  };

/// @brief Analyse une ligne de commande ; argv[0] est ajoute automatiquement.
/// @param p_arguments Arguments apres le nom du programme.
/// @param p_options   Options remplies en sortie.
/// @return Statut de parse_arguments.
int parse( const ::std::vector< ::std::string >& p_arguments, CliOptions& p_options )
  {
  ::std::vector< ::std::string > full_arguments;
  full_arguments.push_back( "line_detector" );
  full_arguments.insert( full_arguments.end(), p_arguments.begin(), p_arguments.end() );

  ArgumentVector argument_vector( full_arguments );
  const int status = parse_arguments( argument_vector.count(), argument_vector.values(), p_options );
  return status;
  }

/// @brief Indique si le message d'erreur contient le fragment attendu.
bool error_mentions( const CliOptions& p_options, const ::std::string& p_fragment )
  {
  const bool found = ( ::std::string::npos != p_options.error_message.find( p_fragment ) );
  return found;
  }

} // namespace

// --- Formes valides ---------------------------------------------------------

TEST_CASE( "parse_arguments : image --file" )
{
  CliOptions options;

  const int status = parse( { "image", "--file", "img_piste/straight.jpg" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( MEDIA_KIND_IMAGE == options.media_kind );
  CHECK( SOURCE_KIND_FILE == options.source_kind );
  CHECK( ::std::string( "img_piste/straight.jpg" ) == options.input_path );
  CHECK( false == options.record );
  CHECK( options.error_message.empty() );
}

TEST_CASE( "parse_arguments : video --file" )
{
  CliOptions options;

  const int status = parse( { "video", "--file", "essai.avi" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( MEDIA_KIND_VIDEO == options.media_kind );
  CHECK( SOURCE_KIND_FILE == options.source_kind );
  CHECK( ::std::string( "essai.avi" ) == options.input_path );
}

TEST_CASE( "parse_arguments : video --camera sans index -> index par defaut" )
{
  CliOptions options;

  const int status = parse( { "video", "--camera" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( MEDIA_KIND_VIDEO == options.media_kind );
  CHECK( SOURCE_KIND_CAMERA == options.source_kind );
  CHECK( DEFAULT_CAMERA_INDEX == options.camera_index );
}

TEST_CASE( "parse_arguments : video --camera avec index explicite" )
{
  CliOptions options;

  const int status = parse( { "video", "--camera", "2" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( SOURCE_KIND_CAMERA == options.source_kind );
  CHECK( 2 == options.camera_index );
}

TEST_CASE( "parse_arguments : video --camera --record -> --record n'est pas un index" )
{
  CliOptions options;

  const int status = parse( { "video", "--camera", "--record" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( SOURCE_KIND_CAMERA == options.source_kind );
  CHECK( DEFAULT_CAMERA_INDEX == options.camera_index );
  CHECK( true == options.record );
}

TEST_CASE( "parse_arguments : video --gstreamer conserve la partie source telle quelle" )
{
  CliOptions options;

  const int status = parse( { "video", "--gstreamer", "libcamerasrc ! video/x-raw,width=1280,height=720" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( MEDIA_KIND_VIDEO == options.media_kind );
  CHECK( SOURCE_KIND_GSTREAMER == options.source_kind );
  CHECK( ::std::string( "libcamerasrc ! video/x-raw,width=1280,height=720" ) == options.gstreamer_pipeline );
}

TEST_CASE( "parse_arguments : --record avant ou apres la source" )
{
  CliOptions image_options;
  const int image_status = parse( { "image", "--file", "a.jpg", "--record" }, image_options );

  CliOptions video_options;
  const int video_status = parse( { "video", "--record", "--file", "b.avi" }, video_options );

  CliOptions camera_options;
  const int camera_status = parse( { "video", "--camera", "1", "--record" }, camera_options );

  CHECK( EXIT_SUCCESS == image_status );
  CHECK( true == image_options.record );
  CHECK( EXIT_SUCCESS == video_status );
  CHECK( true == video_options.record );
  CHECK( SOURCE_KIND_FILE == video_options.source_kind );
  CHECK( EXIT_SUCCESS == camera_status );
  CHECK( true == camera_options.record );
  CHECK( 1 == camera_options.camera_index );
}

// --- Sous-commande ----------------------------------------------------------

TEST_CASE( "parse_arguments : aucun argument -> sous-commande manquante" )
{
  CliOptions options;

  const int status = parse( {}, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Sous-commande manquante" ) );
}

TEST_CASE( "parse_arguments : source sans sous-commande -> sous-commande manquante" )
{
  CliOptions options;

  const int status = parse( { "--file", "a.jpg" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Sous-commande manquante" ) );
}

TEST_CASE( "parse_arguments : ancienne syntaxe --image rejetee (pas d'alias)" )
{
  CliOptions options;

  const int status = parse( { "--image", "a.jpg" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Sous-commande manquante" ) );
}

TEST_CASE( "parse_arguments : sous-commande inconnue" )
{
  CliOptions options;

  const int status = parse( { "photo", "--file", "a.jpg" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Sous-commande inconnue : photo" ) );
}

// --- Source -----------------------------------------------------------------

TEST_CASE( "parse_arguments : aucune source -> source manquante" )
{
  CliOptions bare_options;
  const int bare_status = parse( { "video" }, bare_options );

  CliOptions record_options;
  const int record_status = parse( { "video", "--record" }, record_options );

  CHECK( EXIT_FAILURE == bare_status );
  CHECK( error_mentions( bare_options, "Source manquante" ) );
  CHECK( EXIT_FAILURE == record_status );
  CHECK( error_mentions( record_options, "Source manquante" ) );
}

TEST_CASE( "parse_arguments : deux sources -> sources exclusives" )
{
  CliOptions options;

  const int status = parse( { "video", "--file", "a.mp4", "--camera" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Sources exclusives" ) );
}

TEST_CASE( "parse_arguments : image --camera -> non disponible avec image" )
{
  CliOptions options;

  const int status = parse( { "image", "--camera" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "--camera n'est pas disponible avec image" ) );
}

TEST_CASE( "parse_arguments : image --gstreamer -> non disponible avec image" )
{
  CliOptions options;

  const int status = parse( { "image", "--gstreamer", "videotestsrc" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "--gstreamer n'est pas disponible avec image" ) );
}

// --- Valeurs ----------------------------------------------------------------

TEST_CASE( "parse_arguments : --file sans valeur -> valeur manquante" )
{
  CliOptions options;

  const int status = parse( { "video", "--file" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Valeur manquante apres : --file" ) );
}

TEST_CASE( "parse_arguments : --gstreamer suivi d'un flag -> valeur manquante" )
{
  CliOptions options;

  const int status = parse( { "video", "--gstreamer", "--record" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Valeur manquante apres : --gstreamer" ) );
}

TEST_CASE( "parse_arguments : --file vide -> valeur manquante" )
{
  CliOptions options;

  const int status = parse( { "video", "--file", "" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Valeur manquante apres : --file" ) );
}

TEST_CASE( "parse_arguments : --gstreamer blanc -> valeur manquante" )
{
  CliOptions options;

  const int status = parse( { "video", "--gstreamer", "   " }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Valeur manquante apres : --gstreamer" ) );
}

TEST_CASE( "parse_arguments : --gstreamer avec appsink -> pipeline complet refuse" )
{
  CliOptions options;

  const int status =
    parse( { "video", "--gstreamer", "videotestsrc ! videoconvert ! video/x-raw,format=BGR ! appsink" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "sans element sink" ) );
}

TEST_CASE( "parse_arguments : --gstreamer avec un autre sink -> pipeline complet refuse" )
{
  // Commande de verification du README collee telle quelle : bloquerait l'ouverture.
  CliOptions fakesink_options;
  const int fakesink_status = parse( { "video", "--gstreamer", "videotestsrc ! videoconvert ! fakesink" }, fakesink_options );

  CliOptions filesink_options;
  const int filesink_status =
    parse( { "video", "--gstreamer", "videotestsrc ! filesink location=/tmp/o" }, filesink_options );

  CHECK( EXIT_FAILURE == fakesink_status );
  CHECK( error_mentions( fakesink_options, "sans element sink" ) );
  CHECK( EXIT_FAILURE == filesink_status );
  CHECK( error_mentions( filesink_options, "sans element sink" ) );
}

TEST_CASE( "parse_arguments : --gstreamer avec 'sink' hors nom d'element -> accepte" )
{
  CliOptions options;

  const int status = parse( { "video", "--gstreamer", "filesrc location=/data/appsink_run.mkv ! decodebin" }, options );

  CHECK( EXIT_SUCCESS == status );
  CHECK( SOURCE_KIND_GSTREAMER == options.source_kind );
}

TEST_CASE( "parse_arguments : --gstreamer avec element vide -> refuse" )
{
  CliOptions trailing_options;
  const int trailing_status = parse( { "video", "--gstreamer", "videotestsrc num-buffers=3 !" }, trailing_options );

  CliOptions double_options;
  const int double_status = parse( { "video", "--gstreamer", "videotestsrc ! ! queue" }, double_options );

  CHECK( EXIT_FAILURE == trailing_status );
  CHECK( error_mentions( trailing_options, "element vide" ) );
  CHECK( EXIT_FAILURE == double_status );
  CHECK( error_mentions( double_options, "element vide" ) );
}

TEST_CASE( "parse_arguments : index camera non numerique -> echec" )
{
  CliOptions options;

  const int status = parse( { "video", "--camera", "abc" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Index camera invalide : abc" ) );
}

TEST_CASE( "parse_arguments : index camera hors plage -> echec" )
{
  CliOptions options;

  const int status = parse( { "video", "--camera", "99999999999999999999" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Index camera invalide" ) );
}

// --- Arguments en trop ------------------------------------------------------

TEST_CASE( "parse_arguments : flag inconnu -> echec" )
{
  CliOptions options;

  const int status = parse( { "video", "--file", "a.mp4", "--foo" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Flag inconnu : --foo" ) );
}

TEST_CASE( "parse_arguments : argument positionnel en trop -> echec" )
{
  CliOptions options;

  const int status = parse( { "video", "--file", "a.mp4", "extra" }, options );

  CHECK( EXIT_FAILURE == status );
  CHECK( error_mentions( options, "Argument positionnel non supporte : extra" ) );
}
