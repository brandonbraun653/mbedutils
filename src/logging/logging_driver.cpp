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
#include <cstdarg>
#include <mbedutils/assert.hpp>
#include <mbedutils/config.hpp>
#include <mbedutils/drivers/nanoprintf.hpp>
#include <mbedutils/interfaces/irq_intf.hpp>
#include <mbedutils/interfaces/time_intf.hpp>
#include <mbedutils/logging.hpp>
#include <mbedutils/osal.hpp>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using SinkRegistry = etl::array<SinkHandle_rPtr, MBEDUTILS_LOGGING_MAX_SINKS>;
  using LogBuffer    = etl::array<char, MBEDUTILS_LOGGING_BUFFER_SIZE>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Level                      s_log_level;   /**< Current global log level */
  static SinkHandle_rPtr            s_root_sink;   /**< User assigned root sink */
  static LogBuffer                  s_log_buffer;  /**< Formatting buffer for logs */
  static SinkRegistry               s_sink_reg;    /**< Registry of all user logging sinks */
  static osal::mb_recursive_mutex_t s_driver_lock; /**< Lock for changing driver state */
  static osal::mb_recursive_mutex_t s_format_lock; /**< Lock for accessing the log buffer */

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

    osal::buildRecursiveMutexStrategy( s_driver_lock );
    mbed_assert( s_driver_lock != nullptr );

    osal::buildRecursiveMutexStrategy( s_format_lock );
    mbed_assert( s_format_lock != nullptr );
  }


  ErrCode sets_log_level( const Level level )
  {
    osal::lockRecursiveMutex( s_driver_lock );
    s_log_level = level;
    osal::unlockRecursiveMutex( s_driver_lock );

    return ErrCode::ERR_OK;
  }


  ErrCode registerSink( SinkHandle_rPtr &sink )
  {
    constexpr size_t invalidIndex = std::numeric_limits<size_t>::max();

    size_t nullIndex        = invalidIndex;         /* First index that doesn't have a sink registered */
    bool   sinkIsRegistered = false;                /* Indicates if the sink we are registering already exists */
    bool   registryIsFull   = true;                 /* Is the registry full of sinks? */
    auto   result           = ErrCode::ERR_OK; /* Function return code */

    if( osal::tryLockRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
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
          result           = ErrCode::ERR_OK;
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
        else if( sink->open() != ErrCode::ERR_OK )
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

    if( osal::tryLockRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      auto index = get_sink_offset_idx( sink );
      if( index < s_sink_reg.size() )
      {
        s_sink_reg[ index ]->close();
        s_sink_reg[ index ] = nullptr;
        result              = ErrCode::ERR_OK;
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

        result = ErrCode::ERR_OK;
      }

      osal::unlockRecursiveMutex( s_driver_lock );
    }

    return result;
  }


  ErrCode setRootSink( SinkHandle_rPtr &sink )
  {
    ErrCode result = ErrCode::ERR_LOCKED;

    if( osal::tryLockRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      s_root_sink = sink;
      result      = ErrCode::ERR_OK;

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
    if( ( level < s_log_level ) || !message || !length )
    {
      return ErrCode::ERR_FAIL_BAD_ARG;
    }

    if( !osal::tryLockRecursiveMutex( s_driver_lock, MBEDUTILS_LOGGING_DEFAULT_LOCK_TIMEOUT ) )
    {
      return ErrCode::ERR_LOCKED;
    }

    /*-------------------------------------------------------------------------
    Process the message through each sink. At the moment we won't concern
    ourselves if a sink failed to log. What would we do?
    -------------------------------------------------------------------------*/
    for( size_t i = 0; i < s_sink_reg.size(); i++ )
    {
      if( s_sink_reg[ i ] && ( s_sink_reg[ i ]->logLevel >= s_log_level ) )
      {
        s_sink_reg[ i ]->lock();
        s_sink_reg[ i ]->insert( level, message, length );
        s_sink_reg[ i ]->unlock();
      }
    }

    osal::unlockRecursiveMutex( s_driver_lock );
    return ErrCode::ERR_OK;
  }


  ErrCode flog( const Level lvl, const char *const file, const size_t line, const char *fmt, ... )
  {
    /*-------------------------------------------------------------------------
    Input boundary checking
    -------------------------------------------------------------------------*/
    if( ( lvl < s_log_level ) || !file || !fmt )
    {
      return ErrCode::ERR_FAIL_BAD_ARG;
    }

    /*-------------------------------------------------------------------------
    We can't be in an ISR context when logging due to synchronizing access
    with mutexes. If ISR logging becomes a requirement, a separate format
    buffer and queueing mechanism would be needed + some periodic processing
    to pull the queued messages and log them to the actual sinks in a non-ISR
    context.
    -------------------------------------------------------------------------*/
    if( mb::irq::in_isr() )
    {
      return ErrCode::ERR_FAIL_ISR_CONTEXT;
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
    ErrCode error = ErrCode::ERR_OK;
    osal::lockRecursiveMutex( s_driver_lock );
    {
      s_log_buffer.fill( 0 );
      npf_snprintf( s_log_buffer.data(), s_log_buffer.max_size(),
                    "%u | %s:%u | %s | ",
                    time::millis(), file, line, str_level.data() );

      /*-----------------------------------------------------------------------
      Format the user message
      -----------------------------------------------------------------------*/
      const size_t offset = strlen( s_log_buffer.data() );

      va_list argptr;
      va_start( argptr, fmt );
      npf_vsnprintf( s_log_buffer.data() + offset, s_log_buffer.max_size() - offset, fmt, argptr );
      va_end( argptr );

      /*-----------------------------------------------------------------------
      Ensure the message terminates with a carriage return and newline
      -----------------------------------------------------------------------*/
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

      /*-----------------------------------------------------------------------
      Log through the standard method
      -----------------------------------------------------------------------*/
      error = log( lvl, s_log_buffer.data(), strlen( s_log_buffer.data() ) );
    }
    osal::unlockRecursiveMutex( s_driver_lock );

    return error;
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

}    // namespace mb::logging
