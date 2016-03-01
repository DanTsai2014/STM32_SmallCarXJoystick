#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "EPW_behavior.h"
#include "uart.h"
#include "PWM.h"

void vApplicationTickHook(void) {
}
/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
        taskDISABLE_INTERRUPTS();
        for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
        (void) pcTaskName;
        (void) pxTask;
        /* Run time stack overflow checking is performed if
           configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
           function is called if a stack overflow is detected. */
        taskDISABLE_INTERRUPTS();
        for(;;);
}

void Usart3_Printf(char *string){
    while(*string){
        // send string to USART3 
        USART_SendData(USART3, (unsigned short int) *string++);//unsigned short int

        // wait for sending string finished 
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    }
}


u16 readADC1(u8 channel) { //u16 = unsigned char
	 ADC_SoftwareStartConv(ADC1);//Start the conversion
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	 return ADC_GetConversionValue(ADC1); //Return the converted data
}


int main(void) {

		uint8_t ret = pdFALSE;
    int i;
    int count;
    char buff_x [] = "";
    char buff_y [] = "";
    char buf[20];
    int Working_Time = 50000;
    int Work_Count = 0;

    /*init.*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		init_USART3(9600);

        //init_linear_actuator();
        init_ADC();
        init_Wheels();
        init_Timer();
        init_PWM();
        ADC_SoftwareStartConv(ADC1);

        /*while (1)
        {
            sprintf(buff_x, "ADC_x: %d\n\r", ADC1ConvertedVoltage[0]);
            Usart3_Printf(buff_x); // send string to USART3
            sprintf(buff_y, "ADC_y: %d\n\r", ADC1ConvertedVoltage[1]);
            Usart3_Printf(buff_y);
            for(i=0; i<3000000; i++); // delay
        }*/
        /*unit testing.*/
        /*if(unit_tests_task()){ //unit tests not pass.
           GPIO_WriteBit(GPIOD,GPIO_Pin_14,SET); 
           return 0;
        }else{ //unit tests passed
           //response the success state to user.
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,SET);
           delay(1000);
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,RESET);
        }

		/*create the task. */         
        printf("Task creating...........\r\n");
		//ret = xTaskCreate(neural_task, "neural PID update task", 8192 /*configMINIMAL_STACK_SIZE*/, NULL, 2, NULL);
        //ret &= xTaskCreate(receive_task, "receive command task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 3, NULL);
        //ret &= xTaskCreate(send_data_task, "send data task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
        ret &= xTaskCreate(parse_Joystick_dir, ( signed char * ) "parse Joystick direction", configMINIMAL_STACK_SIZE,NULL, 1, NULL);
		//ret &= xTaskCreate(send_out_task, "send out information task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		//if (ret == pdTRUE) {
				//printf("All tasks are created.\r\n");
                //printf("System Started!\r\n");
				vTaskStartScheduler();  // should never return
		//} else {
				//printf("System Error!\r\n");
				// --TODO blink some LEDs to indicates fatal system error
		//}

		//for (;;);

        return 0;
}





