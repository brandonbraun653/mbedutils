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
#include <etl/pool.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/thread.hpp>
#include <task.h>

/*-----------------------------------------------------------------------------
Validate user configuration parameters are in the project's FreeRTOSConfig.h
-----------------------------------------------------------------------------*/

#if !defined( configMBEDUTILS_MAX_NUM_TASKS ) || ( configMBEDUTILS_MAX_NUM_TASKS <= 0 )
#error "configMBEDUTILS_MAX_NUM_TASKS must be defined as > 0 in FreeRTOSConfig.h"
#endif

namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct FreeRtosTaskMeta
  {
    TaskHandle_t  handle;
    StaticTask_t *task;
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static mb::osal::mb_mutex_t                                       s_task_mutex;
  static etl::pool<StaticTask_t, configMBEDUTILS_MAX_NUM_TASKS>     s_task_pool;
  static etl::pool<FreeRtosTaskMeta, configMBEDUTILS_MAX_NUM_TASKS> s_task_meta_pool;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    mbed_assert( mb::osal::buildMutexStrategy( s_task_mutex ) );
  }

  TaskHandle create_task( const TaskConfig &cfg )
  {
    /*---------------------------------------------------------------------------
    Input validation
    ---------------------------------------------------------------------------*/
    if( ( cfg.func == nullptr ) || ( cfg.name.empty() ) )
    {
      return nullptr;
    }

    /*-------------------------------------------------------------------------
    Allocate a new task meta structure
    -------------------------------------------------------------------------*/
    mbed_dbg_assert( s_task_mutex != nullptr );
    mb::thread::RecursiveLockGuard _lock( s_task_mutex );

    FreeRtosTaskMeta *meta = s_task_meta_pool.allocate();
    if( meta == nullptr )
    {
      mbed_assert_continue_msg( false, "FreeRTOS task meta pool is full" );
      return nullptr;
    }

    meta->handle = nullptr;
    meta->task   = nullptr;

    /*-------------------------------------------------------------------------
    Create a dynamic task
    -------------------------------------------------------------------------*/
    if constexpr( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
    {
      if( ( cfg.stack_size > 0 ) && ( cfg.stack_buf == nullptr ) )
      {
        if( ( cfg.affinity < 0 ) || ( configUSE_CORE_AFFINITY != 1 ) )
        {
          auto result = xTaskCreate( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority, &meta->handle );
          if( result != pdPASS )
          {
            meta->handle = nullptr;
          }
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else
        {
          auto result = xTaskCreateAffinitySet( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                                cfg.affinity, &meta->handle );
          if( result != pdPASS )
          {
            meta->handle = nullptr;
          }
        }
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */
      }
    }

    /*-------------------------------------------------------------------------
    Create a static task
    -------------------------------------------------------------------------*/
    if constexpr( configSUPPORT_STATIC_ALLOCATION == 1 )
    {
      if( ( cfg.stack_buf != nullptr ) && ( cfg.stack_size > 0 ) )
      {
        meta->task = s_task_pool.allocate();

        if( cfg.affinity < 0 )
        {
          meta->handle = xTaskCreateStatic( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                            cfg.stack_buf, meta->task );
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else
        {
          meta->handle = xTaskCreateStaticAffinitySet( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                                       cfg.stack_buf, meta->task, cfg.affinity );
        }
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */
      }
    }

    return static_cast<void *>( meta );
  }


  void destroy_task( TaskHandle task )
  {
    /*-------------------------------------------------------------------------
    Input validation
    -------------------------------------------------------------------------*/
    if( task == nullptr )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Ensure exclusive access to the task pools
    -------------------------------------------------------------------------*/
    mbed_dbg_assert( s_task_mutex != nullptr );
    mb::thread::RecursiveLockGuard _lock( s_task_mutex );

    /*-------------------------------------------------------------------------
    All task resources are stored in the meta structure, so we can just
    delete the task from the FreeRTOS perspective and then release the
    meta structure back to the pool.
    -------------------------------------------------------------------------*/
    auto meta = reinterpret_cast<FreeRtosTaskMeta *>( task );
    if( s_task_meta_pool.is_in_pool( meta ) )
    {
      /* FreeRTOS task deletion */
      if( meta->handle != nullptr )
      {
        vTaskDelete( meta->handle );
      }

      /* If the task was statically allocated, this will need to be released as well */
      if( meta->task && s_task_pool.is_in_pool( meta->task ) )
      {
        s_task_pool.release( meta->task );
      }

      s_task_meta_pool.release( meta );
    }
  }

  void set_affinity( TaskHandle task, size_t coreId )
  {
  }


  void start_scheduler()
  {
    vTaskStartScheduler();
  }
}    // namespace mb::thread::intf

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


  Task &Task::operator=( Task &&other ) noexcept
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

}    // namespace mb::thread
