/******************************************************************************
 *  File Name:
 *    logging_sink_tsdb.cpp
 *
 *  Description:
 *    TimeSeries Database Sink Implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <flashdb.h>
#include <mbedutils/interfaces/time_intf.hpp>
#include <mbedutils/assert.hpp>
#include <mbedutils/logging.hpp>

#include <flashdb.h>

namespace mb::logging
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct ReaderArgs
  {
    TSDBSink::LogReader visitor;
    fdb_tsdb_t          db;
    void               *output;
    size_t              size;
  };

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  static fdb_time_t callback_fdb_get_time()
  {
    static_assert( sizeof( fdb_time_t ) == sizeof( int64_t ), "fdb_cfg.h must define FDB_USING_TIMESTAMP_64BIT" );
    return static_cast<fdb_time_t>( mb::time::micros() );
  }


  static bool callback_fdb_tsl_iter( fdb_tsl_t tsl, void *arg )
  {
    mbed_dbg_assert( tsl != nullptr );
    mbed_dbg_assert( arg != nullptr );

    ReaderArgs *args = static_cast<ReaderArgs *>( arg );
    fdb_blob    blob;
    fdb_blob_t  p_blob   = fdb_tsl_to_blob( tsl, fdb_blob_make( &blob, args->output, args->size ) );
    size_t      act_size = fdb_blob_read( ( fdb_db_t )args->db, p_blob );

    return args->visitor( args->output, act_size );
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  TSDBSink::TSDBSink() : SinkInterface(), mInitialized( false ), mDB( { 0 } ), mReaderBuffer( nullptr )
  {
  }


  void TSDBSink::configure( const Config &config )
  {
    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    mInitialized = false;

    /* clang-format off */
    if( !mbed_assert_continue( config.dev_name.size() > 0 ) ||
        !mbed_assert_continue( config.part_name.size() > 0 ) ||
        !mbed_assert_continue( config.max_log_size > 0 ) )
    { /* clang-format on */
      return;
    }

    /*-------------------------------------------------------------------------
    Configure the database
    -------------------------------------------------------------------------*/
    fdb_err_t result = fdb_tsdb_init( &mDB, config.dev_name.c_str(), config.part_name.c_str(), callback_fdb_get_time,
                                      config.max_log_size, NULL );
    mInitialized     = ( result == FDB_NO_ERR );
    mReaderBuffer    = config.reader_buffer;
    mbed_assert_continue_msg( mInitialized, "Failed to initialize TSDB: %d", result );
  }


  ErrCode TSDBSink::open()
  {
    return mInitialized ? ErrCode::ERR_OK : ErrCode::ERR_FAIL;
  }


  ErrCode TSDBSink::close()
  {
    if( mInitialized )
    {
      fdb_err_t result = fdb_tsdb_deinit( &mDB );
      mInitialized     = false;
      mReaderBuffer    = nullptr;
      mbed_assert_continue_msg( result == FDB_NO_ERR, "Failed to deinitialize TSDB: %d", result );
    }

    return ErrCode::ERR_OK;
  }


  ErrCode TSDBSink::flush()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode TSDBSink::insert( const Level level, const void *const message, const size_t length )
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if ( !enabled || ( level < logLevel ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Write the log to the database
    -------------------------------------------------------------------------*/
    fdb_blob  blob;
    fdb_err_t result  = fdb_tsl_append( &mDB, fdb_blob_make( &blob, message, length ) );
    bool      success = ( result == FDB_NO_ERR );

    mbed_assert_continue_msg( success, "Failed to insert log into TSDB: %d", result );
    return success ? ErrCode::ERR_OK : ErrCode::ERR_FAIL;
  }


  void TSDBSink::read( LogReader visitor, const bool direction )
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Input Validation
    -------------------------------------------------------------------------*/
    if( !mInitialized || !visitor  || !mReaderBuffer )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Read the logs from the database
    -------------------------------------------------------------------------*/
    auto       read_func = direction ? fdb_tsl_iter : fdb_tsl_iter_reverse;
    ReaderArgs args      = { visitor, &mDB, mReaderBuffer, mDB.max_len };

    read_func( &mDB, callback_fdb_tsl_iter, &args );
  }
}  // namespace mb::logging
