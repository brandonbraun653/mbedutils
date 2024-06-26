/******************************************************************************
 *  File Name:
 *    logging_driver.cpp
 *
 *  Description:
 *    Logging utilities implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/config.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/osal.hpp>

/*-----------------------------------------------------------------------------
Configuration Options
-----------------------------------------------------------------------------*/

#if !defined( MBEDUTILS_LOGGING_MAX_SINKS )
#define MBEDUTILS_LOGGING_MAX_SINKS ( 4 )
#endif

#if !defined( MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT )
#define MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ( 100 )
#endif

#if !defined( MBEDUTILS_LOGGING_BUFFER_SIZE )
#define MBEDUTILS_LOGGING_BUFFER_SIZE ( 512 )
#endif

namespace mbedutils::logging
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using SinkRegistry = etl::array<SinkHandle_rPtr, MBEDUTILS_LOGGING_MAX_SINKS>;
  using LogBuffer    = etl::array<char, MBEDUTILS_LOGGING_BUFFER_SIZE>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Level                     s_log_level;   /**< Current global log level */
  static SinkHandle_rPtr           s_root_sink;   /**< User assigned root sink */
  static LogBuffer                 s_log_buffer;  /**< Formatting buffer for logs */
  static SinkRegistry              s_sink_reg;    /**< Registry of all user logging sinks */
  static osal::recursive_mutex_t   s_driver_lock; /**< Lock for changing driver state */
  static osal::recursive_mutex_t   s_format_lock; /**< Lock for accessing the log buffer */

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Looks up the registry index associated with a particular sink handle
   *
   * @param sinkHandle  The handle to search for
   * @return size_t
   */
  static size_t get_sink_offset_idx( const SinkHandle_rPtr &sinkHandle );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Prepare the module memory
    -------------------------------------------------------------------------*/
    s_log_level = Level::LVL_MIN;
    s_root_sink = nullptr;
    s_log_buffer.fill( 0 );
    s_sink_reg.fill( nullptr );

    osal::createRecursiveMutex( &s_driver_lock );
    assert( s_driver_lock != nullptr );
    osal::initializeRecursiveMutex( s_driver_lock );

    osal::createRecursiveMutex( &s_format_lock );
    assert( s_format_lock != nullptr );
    osal::initializeRecursiveMutex( s_format_lock );
  }


  ErrCode sets_log_level( const Level level )
  {
    osal::lockRecursiveMutex( s_driver_lock );
    s_log_level = level;
    osal::unlockRecursiveMutex( s_driver_lock );

    return ErrCode::ERR_SUCCESS;
  }


  ErrCode registerSink( SinkHandle_rPtr &sink )
  {
    constexpr size_t invalidIndex = std::numeric_limits<size_t>::max();

    size_t nullIndex        = invalidIndex;         /* First index that doesn't have a sink registered */
    bool   sinkIsRegistered = false;                /* Indicates if the sink we are registering already exists */
    bool   registryIsFull   = true;                 /* Is the registry full of sinks? */
    auto   result           = ErrCode::ERR_SUCCESS; /* Function return code */

    if( osal::tryLockForRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      /*-----------------------------------------------------------------------
      Check if the sink already is registered + an empty slot to insert at.
      -----------------------------------------------------------------------*/
      for( size_t i = 0; i < s_sink_reg.size(); i++ )
      {
        /* Did we find the first location that is free? */
        if( ( nullIndex == invalidIndex ) && ( s_sink_reg[ i ] == nullptr ) )
        {
          nullIndex      = i;
          registryIsFull = false;
        }

        /* Does the sink already exist in the registry? */
        if( s_sink_reg[ i ] == sink )
        {
          sinkIsRegistered = true;
          registryIsFull   = false;
          result           = ErrCode::ERR_SUCCESS;
          break;
        }
      }

      /*-----------------------------------------------------------------------
      Perform the registration
      -----------------------------------------------------------------------*/
      if( !sinkIsRegistered )
      {
        if( registryIsFull )
        {
          result = ErrCode::ERR_FULL;
        }
        else if( sink->open() != ErrCode::ERR_SUCCESS )
        {
          result = ErrCode::ERR_FAIL;
        }
        else
        {
          s_sink_reg[ nullIndex ] = sink;
        }
      }

      osal::unlockRecursiveMutex( s_driver_lock );
    }

    return result;
  }


  ErrCode removeSink( SinkHandle_rPtr &sink )
  {
    ErrCode result = ErrCode::ERR_LOCKED;

    if( osal::tryLockForRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      auto index = get_sink_offset_idx( sink );
      if( index < s_sink_reg.size() )
      {
        s_sink_reg[ index ]->close();
        s_sink_reg[ index ] = nullptr;
        result              = ErrCode::ERR_SUCCESS;
      }
      else if( sink == nullptr )
      {
        for( size_t i = 0; i < s_sink_reg.size(); i++ )
        {
          if( s_sink_reg[ i ] )
          {
            s_sink_reg[ i ]->close();
            s_sink_reg[ i ] = nullptr;
          }
        }

        result = ErrCode::ERR_SUCCESS;
      }

      osal::unlockRecursiveMutex( s_driver_lock );
    }

    return result;
  }


  ErrCode setRootSink( SinkHandle_rPtr &sink )
  {
    ErrCode result = ErrCode::ERR_LOCKED;

    if( osal::tryLockForRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      s_root_sink = sink;
      result      = ErrCode::ERR_SUCCESS;

      osal::unlockRecursiveMutex( s_driver_lock );
    }

    return result;
  }


  void getRootSink( SinkHandle_rPtr *handle )
  {
    assert( handle != nullptr );
    *handle = s_root_sink;
  }


  ErrCode log( const Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Input boundary checking
    -------------------------------------------------------------------------*/
    Chimera::Thread::TimedLockGuard x( s_driver_lock );
    if( !x.try_lock_for( MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      return ErrCode::ERR_LOCKED;
    }
    else if( ( level < s_log_level ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Process the message through each sink. At the moment we won't concern
    ourselves if a sink failed to log. What would we do?
    -------------------------------------------------------------------------*/
    for( size_t i = 0; i < s_sink_reg.size(); i++ )
    {
      if( s_sink_reg[ i ] && ( s_sink_reg[ i ]->logLevel >= s_log_level ) )
      {
        s_sink_reg[ i ]->log( level, message, length );
      }
    }

    return ErrCode::ERR_SUCCESS;
  }


  ErrCode flog( const Level lvl, const char *const file, const size_t line, const char *fmt, ... )
  {
    /*-------------------------------------------------------------------------
    Input boundary checking
    -------------------------------------------------------------------------*/
    if( ( lvl < s_log_level ) || !file || !fmt )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Create the logging level
    -------------------------------------------------------------------------*/
    std::string_view str_level = "";
    switch( lvl )
    {
      case Level::LVL_TRACE:
        str_level = "TRACE";
        break;

      case Level::LVL_DEBUG:
        str_level = "DEBUG";
        break;

      case Level::LVL_INFO:
        str_level = "INFO";
        break;

      case Level::LVL_WARN:
        str_level = "WARN";
        break;

      case Level::LVL_ERROR:
        str_level = "ERROR";
        break;

      case Level::LVL_FATAL:
        str_level = "FATAL";
        break;

      default:
        return ErrCode::ERR_INVALID_LEVEL;
    };

    /*-------------------------------------------------------------------------
    Format the message header
    -------------------------------------------------------------------------*/
    // TODO: Add assertion for detecting that we're not in an ISR
    Chimera::Thread::LockGuard _lock( s_format_lock );

    s_log_buffer.fill( 0 );
    npf_snprintf( s_log_buffer, s_log_buffer.max_size(), "%u | %s:%u | %s | ", Chimera::millis(), file, line, str_level.data() );

    /*-------------------------------------------------------------------------
    Format the user message
    -------------------------------------------------------------------------*/
    const size_t offset = strlen( s_log_buffer.data() );

    va_list argptr;
    va_start( argptr, fmt );
    npf_vsnprintf( s_log_buffer.data() + offset, s_log_buffer.max_size() - offset, fmt, argptr );
    va_end( argptr );

    /*-------------------------------------------------------------------------
    Ensure the message terminates with a carriage return and newline
    -------------------------------------------------------------------------*/
    const size_t msg_len = strlen( s_log_buffer.data() );

    bool ends_with_crlf = false;
    if( ( msg_len >= 2 ) && ( msg_len < s_log_buffer.max_size() ) )
    {
      if( s_log_buffer[ msg_len - 2 ] == '\r' && s_log_buffer[ msg_len - 1 ] == '\n' )
      {
        ends_with_crlf = true;
      }
      else if( s_log_buffer[ msg_len - 2 ] == '\n' && s_log_buffer[ msg_len - 1 ] == '\r' )
      {
        ends_with_crlf = true;
      }
    }

    if( !ends_with_crlf )
    {
      if( msg_len < s_log_buffer.max_size() - 2 )
      {
        s_log_buffer[ msg_len ]     = '\r';
        s_log_buffer[ msg_len + 1 ] = '\n';
      }
      else
      {
        s_log_buffer[ s_log_buffer.max_size() - 2 ] = '\r';
        s_log_buffer[ s_log_buffer.max_size() - 1 ] = '\n';
      }
    }

    /*-------------------------------------------------------------------------
    Log through the standard method
    -------------------------------------------------------------------------*/
    return log( lvl, s_log_buffer.data(), strlen( s_log_buffer.data() ) );
  }


  static size_t get_sink_offset_idx( const SinkHandle_rPtr &sinkHandle )
  {
    /*-------------------------------------------------------------------------
    Figure out the real addresses and boundary limit them
    Note: I'm going to run into trouble if packing is weird...
    -------------------------------------------------------------------------*/
    std::uintptr_t offsetAddress = reinterpret_cast<std::uintptr_t>( &sinkHandle );
    std::uintptr_t beginAddress  = reinterpret_cast<std::uintptr_t>( &s_sink_reg[ 0 ] );
    std::uintptr_t secondAddress = reinterpret_cast<std::uintptr_t>( &s_sink_reg[ 1 ] );
    size_t         elementSize   = secondAddress - beginAddress;

    if( ( sinkHandle == nullptr ) || ( beginAddress > offsetAddress ) || !elementSize )
    {
      return std::numeric_limits<size_t>::max();
    }

    /*-------------------------------------------------------------------------
    Calculate the index
    -------------------------------------------------------------------------*/
    size_t index = ( offsetAddress - beginAddress ) / elementSize;

    if( index > s_sink_reg.size() )
    {
      index = std::numeric_limits<size_t>::max();
    }

    return index;
  }

}    // namespace mbedutils::logging
