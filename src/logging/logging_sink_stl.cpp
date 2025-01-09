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
#include "mbedutils/drivers/logging/logging_sinks.hpp"
#include "mbedutils/drivers/logging/logging_types.hpp"
#include "mbedutils/drivers/threading/lock.hpp"
#include <filesystem>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <mbedutils/logging.hpp>
#include <mbedutils/interfaces/mutex_intf.hpp>
#include <string>
#include <fstream>
#include <vector>


namespace mb::logging
{
  /*---------------------------------------------------------------------------
  ConsoleSink
  ---------------------------------------------------------------------------*/

  ErrCode ConsoleSink::open()
  {
    this->initLockable();
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

  ErrCode ConsoleSink::write( const Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if( !enabled || ( level < logLevel ) || !message || !length )
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
    if( mFileHandle >= 0 )
    {
      ::close( mFileHandle );
      mFileHandle = -1;
    }
  }


  ErrCode STLFileSink::open()
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Check to see if someone has configured the filename yet
    -------------------------------------------------------------------------*/
    if( mFile.empty() )
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

    if( mFileHandle < 0 )
    {
      return ErrCode::ERR_FAIL;
    }

    return ErrCode::ERR_OK;
  }


  ErrCode STLFileSink::close()
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    if( mFileHandle >= 0 )
    {
      ::close( mFileHandle );
      mFileHandle = -1;
    }

    return ErrCode::ERR_OK;
  }


  ErrCode STLFileSink::flush()
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    if( mFileHandle >= 0 )
    {
      ::fsync( mFileHandle );
    }

    return ErrCode::ERR_OK;
  }


  ErrCode STLFileSink::erase()
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Ensure we're in an open state
    -------------------------------------------------------------------------*/
    if( mFileHandle < 0 )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Close the file, delete it, then reopen it
    -------------------------------------------------------------------------*/
    this->close();
    fs::remove( mFile.c_str() );
    return this->open();
  }


  ErrCode STLFileSink::write( const Level level, const void *const message, const size_t length )
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );

    /*-------------------------------------------------------------------------
    Check to see if we should even write
    -------------------------------------------------------------------------*/
    if( !enabled || ( level < logLevel ) || !message || !length )
    {
      return ErrCode::ERR_FAIL;
    }

    /*-------------------------------------------------------------------------
    Write the message to the file
    -------------------------------------------------------------------------*/
    if( mFileHandle >= 0 )
    {
      ::write( mFileHandle, message, length );
      return ErrCode::ERR_OK;
    }
    else
    {
      return ErrCode::ERR_FAIL;
    }
  }


  void STLFileSink::read( LogReader visitor, const bool direction )
  {
    thread::RecursiveLockGuard lock( this->mLockableMutex );
    if( mFileHandle < 0 )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Close and commit any pending writes. Temporary disable the sink to prevent
    any writes while we're reading.
    -------------------------------------------------------------------------*/
    enabled = false;
    this->close();

    /*-------------------------------------------------------------------------
    Read the file contents in the specified direction
    -------------------------------------------------------------------------*/
    std::ifstream fileStream( mFile.c_str(), std::ios::in | std::ios::binary );
    if( !fileStream.is_open() )
    {
      return;
    }

    std::string line;
    if( direction )    // Oldest to newest
    {
      /*-----------------------------------------------------------------------
      Iterate over the lines in the file
      -----------------------------------------------------------------------*/
      while( std::getline( fileStream, line ) )
      {
        if( !visitor( line.c_str(), line.size() ) )
        {
          break;
        }
      }
    }
    else    // Newest to oldest
    {
      /*-----------------------------------------------------------------------
      Read the file into memory
      -----------------------------------------------------------------------*/
      std::vector<std::string> lines;
      while( std::getline( fileStream, line ) )
      {
        lines.push_back( line );
      }

      /*-----------------------------------------------------------------------
      Iterate over the lines in reverse order
      -----------------------------------------------------------------------*/
      for( auto it = lines.rbegin(); it != lines.rend(); ++it )
      {
        if( !visitor( it->c_str(), it->size() ) )
        {
          break;
        }
      }
    }

    /*-------------------------------------------------------------------------
    Reopen the file and re-enable the sink
    -------------------------------------------------------------------------*/
    this->open();
    enabled = true;
  }


  void STLFileSink::setFile( const etl::string_view &file )
  {
    this->initLockable();
    thread::RecursiveLockGuard lock( this->mLockableMutex );
    mFile.assign( file.begin(), file.end() );
  }
}    // namespace mb::logging
