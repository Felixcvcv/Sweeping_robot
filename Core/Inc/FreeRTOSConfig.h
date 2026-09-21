/*
 * FreeRTOS 内核配置文件
 * 目标：STM32F103C8T6（Cortex-M3，4 位中断优先级，主频 72MHz）
 * 编译器：Keil MDK-ARM（ARMCC / ARM Compiler 5），移植层使用 portable/RVDS/ARM_CM3
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* SystemCoreClock 在 system_stm32f1xx.c 中定义，SystemClock_Config() 后为 72MHz */
extern uint32_t SystemCoreClock;

/*---------------------------------------------------------------------------
 * 调度器基本配置
 *-------------------------------------------------------------------------*/
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                  0
#define configUSE_TIME_SLICING                   1
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                     5
#define configMINIMAL_STACK_SIZE                 128
#define configMAX_TASK_NAME_LEN                  8
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configQUEUE_REGISTRY_SIZE                0
#define configUSE_QUEUE_SETS                     0
#define configUSE_NEWLIB_REENTRANT               0
#define configENABLE_BACKWARD_COMPATIBILITY      0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
#define configUSE_MINI_LIST_ITEM                 1
#define configSTACK_DEPTH_TYPE                   uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
#define configUSE_APPLICATION_TASK_TAG           0

/*---------------------------------------------------------------------------
 * 内存管理：heap_4（带相邻空闲块合并的动态分配）
 * STM32F103C8T6 只有 20KB RAM，内核堆给 6KB，可满足 4 个任务栈 + 队列 + TCB
 *-------------------------------------------------------------------------*/
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 6 * 1024 ) )
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configHEAP_CLEAR_MEMORY_ON_FREE          1

/*---------------------------------------------------------------------------
 * 钩子函数
 *-------------------------------------------------------------------------*/
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1

/*---------------------------------------------------------------------------
 * 运行统计 / 跟踪（调试时才打开）
 *-------------------------------------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/*---------------------------------------------------------------------------
 * 功能裁剪：本工程不使用软件定时器与协程，节省 Flash / RAM
 *-------------------------------------------------------------------------*/
#define configUSE_TIMERS                         0
#define configUSE_CO_ROUTINES                    0

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           0
#define INCLUDE_eTaskGetState                    0
#define INCLUDE_xTaskAbortDelay                  0
#define INCLUDE_xTaskGetHandle                   0
#define INCLUDE_xTimerPendFunctionCall           0
#define INCLUDE_xQueueGetMutexHolder             0

/*---------------------------------------------------------------------------
 * Cortex-M3 中断优先级
 * 允许调用 ...FromISR() 版本 API 的中断，其优先级数值必须 >= 5（即优先级
 * 低于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
 * 本工程：USART1/2/3、TIM4 优先级 = 5；TIM2 不调用内核 API，可以更高。
 *-------------------------------------------------------------------------*/
#define configPRIO_BITS                             4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY             ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*---------------------------------------------------------------------------
 * 断言：失败时关中断并停在此处，便于调试器定位
 *-------------------------------------------------------------------------*/
#define configASSERT( x )    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/*---------------------------------------------------------------------------
 * 内核异常处理函数与 CMSIS 标准向量名的映射
 * 注意：这里故意“不”映射 xPortSysTickHandler —— SysTick_Handler 仍保留在
 *       stm32f1xx_it.c 中（先 HAL_IncTick() 再调用 xPortSysTickHandler()），
 *       这样才能让 STM32Cube HAL 的 HAL_GetTick()/HAL_Delay() 时基继续工作。
 *-------------------------------------------------------------------------*/
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler

#endif /* FREERTOS_CONFIG_H */
