/***************************************************************************//**
 * @file app.c
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_cmu.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "glib.h"
#include <stdio.h>
#include "sl_board_control.h"
#include "sl_simple_button_instances.h"
#include "em_assert.h"
#include "dmd.h"
#include "font.h"
#include "app.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifndef BUTTON_INSTANCE_0
#define BUTTON_INSTANCE_0   sl_button_btn0
#endif

#ifndef BUTTON_INSTANCE_1
#define BUTTON_INSTANCE_1   sl_button_btn1
#endif


#define tskIDLE_PRIORITY    ( ( UBaseType_t ) 0U )

#ifndef TASK_STACK_SIZE
#define TASK_STACK_SIZE       800
#endif

#ifndef TASK_PRIO
#define TASK_PRIO            20
#endif

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
static GLIB_Context_t glibContext;
static int currentLine = 0;
static osEventFlagsId_t evt_button_id;                        // event flags id

#ifdef STORAGE_EXTERNAL_FLASH
  #ifdef SPI_FLASH_NEED_INITIALISATION
//32x64 font
const GLIB_Font_t GLIB_FontNarrow = { (void *)console_font,
                                      sizeof(console_font),
                                      4,
                                      1, 32, 64, 2, 0, FullFont };
  #else
//32x64 font
const GLIB_Font_t GLIB_FontNarrow = { NULL,
                                      24320,
                                      4,
                                      1, 32, 64, 2, 0, FullFont };
  #endif
#else
//32x64 font
const GLIB_Font_t GLIB_FontNarrow = { (void *)console_font,
                                      sizeof(console_font),
                                      4,
                                      1, 32, 64, 2, 0, FullFont };
#endif


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

int32_t storage_init(void);
void initLDMA(void);


/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/

void memlcd_app_init(void)
{
  uint32_t status;

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow);

  DMD_updateDisplay();
}

void draw_string(char * str, uint32_t x)
{
  GLIB_drawStringOnLine(&glibContext,
                         str,
                         currentLine,
                         GLIB_ALIGN_LEFT,
                         x,
                         32,
                         true);

   DMD_updateDisplay();
}

void clear_display()
{
  GLIB_clear(&glibContext);
  GLIB_drawCircle(&glibContext, 64, 64, 52);
  DMD_updateDisplay();
}

/***************************************************************************//**
 * Ticking function.
 ******************************************************************************/
void memlcd_app_process_action(void)
{
  return;
}

/***************************************************************************//**
 * Callback on button change.
 *
 * This function overrides a weak implementation defined in the simple_button
 * module. It is triggered when the user activates one of the buttons.
 *
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&BUTTON_INSTANCE_0 == handle) {
      currentLine = 0;
      osEventFlagsSet(evt_button_id, 0x0001U);
    } else if (&BUTTON_INSTANCE_1 == handle) {
      osEventFlagsSet(evt_button_id, 0x0002U);
    }

  }
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
static void task(void *arg);

void app_init(void)
{

#ifdef STORAGE_EXTERNAL_FLASH
  /* Flash Init. This does not use LDMA */
  if (storage_init())
    printf("Storage initialisation error\r\n");

  #ifdef SPI_FLASH_NEED_INITIALISATION
    /* Write Font to memory if not already there. This does not use LDMA */
    int32_t ret = storage_writeRaw(0, (uint8_t*)GLIB_FontNarrow.pFontPixMap, GLIB_FontNarrow.cntOfMapElements);
    if (ret == -3)
    {
      printf("Font already in SPI Flash \r\n");
    }
    else if (ret)
    {
      printf("Writing Font error %ld \r\n", ret);
    }
  #endif
#endif

  initLDMA();

  evt_button_id = osEventFlagsNew(NULL);

  TaskHandle_t xHandle = NULL;

  static StaticTask_t xTaskBuffer;
  static StackType_t  xStack[TASK_STACK_SIZE];

  // Create Task without using any dynamic memory allocation
  xHandle = xTaskCreateStatic(task,
                              "task",
                              TASK_STACK_SIZE,
                              ( void * ) NULL,
                              tskIDLE_PRIORITY + 1,
                              xStack,
                              &xTaskBuffer);

  // Since puxStackBuffer and pxTaskBuffer parameters are not NULL,
  // it is impossible for xHandle to be null. This check is for
  // rigorous example demonstration.
  EFM_ASSERT(xHandle != NULL);
}

char str[32];
uint16_t v = 0;

static void task(void *arg)
{
  (void)&arg;

  memlcd_app_init();

  while (1) {
      uint32_t flags = osEventFlagsWait(evt_button_id, 0x0003U, osFlagsWaitAny, osWaitForever);
      if (flags == 0x02)
        {
          if (!v)
            clear_display();

          sprintf(str, "%d", v);
          draw_string(str, v++ > 9 ? 32 : 48);

          if (v== 99)
            v = 0;
        }
      else
        {
          v = 0;
          clear_display();
          sprintf(str, "%d", v);
          draw_string(str, v++ > 9 ? 32 : 48);
        }
  }

}
/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  memlcd_app_process_action();
}
