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
#include <FreeRTOS.h>
#include <mbedutils/thread.hpp>


namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  TaskHandle_t create_task( const TaskConfig &cfg )
  {
    return nullptr;
  }


  void set_affinity( TaskHandle_t task, size_t coreId )
  {
  }


  void start_scheduler()
  {
  }
}

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Task::Task( const TaskId id ) noexcept : taskId( id ), pimpl( nullptr )
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
