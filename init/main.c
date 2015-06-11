/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "system.h"

/* ST includes */
#include "stm32f4xx.h"

void nvicInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}


int main(void)
{
	
	nvicInit();
	
	/* Launch the system task that will initialize and start everything */
  systemLaunch();
	
  /* Æô¶¯OS */
  vTaskStartScheduler();

  return 0;
}

