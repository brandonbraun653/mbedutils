/******************************************************************************
 *  File Name:
 *    logging_sink_stl.cpp
 *
 *  Description:
 *    Log sinks that are designed to work with the STL
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <filesystem>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <mbedutils/logging.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <string>


namespace mb::logging
{
  /*---------------------------------------------------------------------------
  ConsoleSink
  ---------------------------------------------------------------------------*/

  ErrCode ConsoleSink::open()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode ConsoleSink::close()
  {
    return ErrCode::ERR_OK;
  }


  ErrCode ConsoleSink::flush()
  {
    std::cout << std::flush;
    return ErrCode::ERR_OK;
  }

  ErrCode ConsoleSink::insert( const Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if ( !enabled || ( level < logLevel ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Simply assign the data as a string, then write it.
    -------------------------------------------------------------------------*/
    std::string coutBuffer;
    coutBuffer.assign( reinterpret_cast<const char *const>( message ), length );
    std::cout << coutBuffer << std::flush;

    return ErrCode::ERR_OK;
  }


  /*---------------------------------------------------------------------------
  STLFileSink
  ---------------------------------------------------------------------------*/
  namespace fs = std::filesystem;

  STLFileSink::STLFileSink() : mFile( "" ), mFileHandle( -1 )
  {
  }


  STLFileSink::~STLFileSink()
  {
    if ( mFileHandle >= 0 )
    {
      ::close( mFileHandle );
    }
  }


  ErrCode STLFileSink::open()
  {
    osal::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Check to see if someone has configured the filename yet
    -------------------------------------------------------------------------*/
    if ( mFile.empty() )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Open the file for writing
    -------------------------------------------------------------------------*/
    fs::path filePath( mFile.c_str() );
    if( !fs::exists( filePath ) )
    {
      mFileHandle = ::open( mFile.c_str(), O_CREAT | O_WRONLY, 0644 );
    }
    else
    {
      mFileHandle = ::open( mFile.c_str(), O_WRONLY | O_APPEND );
    }

    if ( mFileHandle < 0 )
    {
      return ErrCode::ERR_FAIL;
    }
  }


  ErrCode STLFileSink::close()
  {
    osal::RecursiveLockGuard lock( this->mLockableMutex );

    if ( mFileHandle >= 0 )
    {
      ::close( mFileHandle );
      mFileHandle = -1;
    }

    return ErrCode::ERR_OK;
  }


  ErrCode STLFileSink::flush()
  {
    osal::RecursiveLockGuard lock( this->mLockableMutex );

    if ( mFileHandle >= 0 )
    {
      ::fsync( mFileHandle );
    }

    return ErrCode::ERR_OK;
  }


  ErrCode STLFileSink::insert( const Level level, const void *const message, const size_t length )
  {
    osal::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if ( !enabled || ( level < logLevel ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Write the message to the file
    -------------------------------------------------------------------------*/
    if ( mFileHandle >= 0 )
    {
      ::write( mFileHandle, message, length );
      return ErrCode::ERR_OK;
    }
    else
    {
      return ErrCode::ERR_FAIL;
    }
  }


  void STLFileSink::setFile( const etl::string_view &file )
  {
    osal::RecursiveLockGuard lock( this->mLockableMutex );
    mFile.assign( file.begin(), file.end() );
  }
}  // namespace mb::logging
