/******************************************************************************
 *  File Name:
 *    thread_freertos.cpp
 *
 *  Description:
 *    FreeRTOS based threading implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/thread.hpp>

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Task::Task() noexcept : pimpl( nullptr )
  {
  }


  Task::~Task()
  {
  }


  Task& Task::operator=( Task &&other ) noexcept
  {
    return *this;
  }


  void start()
  {
  }


  void Task::suspend()
  {
  }


  void Task::resume()
  {
  }


  void Task::kill()
  {
  }


  void Task::join()
  {
  }


  bool Task::joinable()
  {
    return false;
  }

}  // namespace mb::thread
