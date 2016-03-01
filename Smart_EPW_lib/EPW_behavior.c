#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "timers.h"
#include "uart.h"
#include "PWM.h"

#define B_phase 1
#define NEURAL_IDENTIFIER 1
#define PID_ADJUST 1
#define RECORD_SIZE 750
int tremor_count = 0;
char buff [] = "";

/*============================================================================*/
/*============================================================================*
 ** function : init_ADC
 ** brief : initialize ADC reading setting
 **param : None
 **retval : None
 **============================================================================*/
 /*============================================================================*/

void init_ADC(void){
	//__IO uint16_t ADC1ConvertedVoltage[2];
	ADC_InitTypeDef ADC_InitStructure; //Structure for adc configuration
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	ADC_StructInit(&ADC_InitStructure);
    ADC_CommonStructInit(&ADC_CommonInitStructure);
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN | RCC_AHB1Periph_DMA2, ENABLE); //Clock for the ADC port!! Do not forget about this one ;)
	//Analog pin configuration
	GPIO_StructInit(&GPIO_initStructre);
	GPIO_initStructre.GPIO_Pin = JOYSTICK_X_AXIS_PIN | JOYSTICK_Y_AXIS_PIN; //The channel 10 is connected to PC0; PC1 if multiple channels
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(JOYSTICK_PORT, &GPIO_initStructre); //Affecting the port with the initialization structure configuration
	//ADC structure configuration
	//ADC_DeInit();
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //data converted will be shifted to right
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; //Input voltage is converted into a 12bit number giving a maximum value of 4095
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // conversion is synchronous with TIM1 and CC1 (use timer 1 capture/compare channel 1 for external trigger)
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //no trigger for conversion
	ADC_InitStructure.ADC_NbrOfConversion = 2; //Number of used ADC channels;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //The scan is configured in muptiple channels
	ADC_Init(ADC1, &ADC_InitStructure); //Initialize ADC with the previous configuration
	

	DMA_InitTypeDef DMA_InitStructure; //Structure for DMA configuration
	DMA_DeInit(DMA2_Stream4);
	DMA_StructInit(&DMA_InitStructure);
    //DMA2 Channel0 stream0 configuration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; //DMA channel
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //DMA address
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //u32 //Peripheral Data Size 32bit (DMA_{PeripheralDataSize_HalfWord 16bit})
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedVoltage; //buffer address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//傳輸方向單向
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA Memory Data Size 32bit
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //接收一次數據後，目標內存地址是否後移--重要概念，用來采集多個數據的，多通道的時候需要使能它
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//接收一次數據後，設備地址是否後移
    DMA_InitStructure.DMA_Mode  = DMA_Mode_Circular;//轉換模式，循環緩存模式，常用
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA優先級，高
    DMA_InitStructure.DMA_BufferSize = 2;//DMA緩存大小，1*2個
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = 0;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //send values to DMA registers
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    // Enable DMA2 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
    //Enable DMA1 Channel transfer
    DMA_Cmd(DMA2_Stream4, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); //// use channel 10 from ADC1, with sample time 144 cycles
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles); //ADC1 multiple channels (channel 11)
	//Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    //Enable using ADC_DMA
    ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);

}

/*============================================================================*/
/*============================================================================*
 ** function : parse_Joystick_dir
 ** brief : parse Joystick direction from the uart siganl
 ** param : Joystick_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/

void parse_Joystick_dir(void *p) //unsigned uint16_t Joystick_cmd
{
	while(1)
	{ //2305: 靜止x軸平均值, 2362: 靜止y軸平均值
	if(ADC1ConvertedVoltage[1] >= 3000 && ADC1ConvertedVoltage[1] - 2362 > ADC1ConvertedVoltage[0] - 2305 && ADC1ConvertedVoltage[1] - 2362 > 2305 - ADC1ConvertedVoltage[0]){ //forward(3000~4095)
		//if (data_sending != 1) /*&& car_state == CAR_STATE_IDLE) // Do not control the wheelchair when sending data with usart!
			//{
				/*Neural_Control*/
				/*
				controller_initialize(&n_r);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_FORWARD;
			}*/
				//sprintf(buff, "forward");
                //Usart3_Printf(buff); // send string to USART3

				//PWM_Control
				TIM_SetCompare1(TIM1, 256 - 1); 
				TIM_SetCompare2(TIM1, 256 - 1);
				TIM_SetCompare1(TIM3, 0);
				TIM_SetCompare2(TIM3, 127); //minimum: 95
				TIM_SetCompare3(TIM3, 0);
				TIM_SetCompare4(TIM3, 127);
				vTaskDelay(1200);
				//reset_Wheels();
				//vTaskDelay(100);
		    }
		else if(ADC1ConvertedVoltage[0] < 3000 && ADC1ConvertedVoltage[1] < 3000 && ADC1ConvertedVoltage[0] > 1500 && ADC1ConvertedVoltage[1] > 1500){  //stop
				//car_state = CAR_STATE_STOPPING;
			//sprintf(buff, "stop");
            //Usart3_Printf(buff); // send string to USART3
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare2(TIM1, 0);
			TIM_SetCompare1(TIM3, 0);
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare3(TIM3, 0);
			TIM_SetCompare4(TIM3, 0);
			vTaskDelay(1200);
			//reset_Wheels();
			//vTaskDelay(100);
		}
		else if(/*data_sending != 1 && */ADC1ConvertedVoltage[1] <= 1500 && 2362 - ADC1ConvertedVoltage[1] > 2305 - ADC1ConvertedVoltage[0] && 2362 - ADC1ConvertedVoltage[1] > ADC1ConvertedVoltage[0] - 2305){  //backward(1500~0)
			/*if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_BACK;
            }*/
				//sprintf(buff, "backward");
                //Usart3_Printf(buff); // send string to USART3
				TIM_SetCompare1(TIM1, 256 - 1);
				TIM_SetCompare2(TIM1, 256 - 1);
				TIM_SetCompare1(TIM3, 127);
				TIM_SetCompare2(TIM3, 0);
				TIM_SetCompare3(TIM3, 127);
				TIM_SetCompare4(TIM3, 0);
				vTaskDelay(1200);
				//reset_Wheels();
				//vTaskDelay(100);
		}
        else if(/*data_sending != 1 && */ADC1ConvertedVoltage[0] >= 3000 && ADC1ConvertedVoltage[0] - 2305 > ADC1ConvertedVoltage[1] - 2362 && ADC1ConvertedVoltage[0] -2305 > 2362 - ADC1ConvertedVoltage[1]){  //left(3000~4095)
        	/*if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_LEFT;
            }*/
                //sprintf(buff, "left");
                //Usart3_Printf(buff); // send string to USART3
                TIM_SetCompare1(TIM1, 256 - 1);
				TIM_SetCompare2(TIM1, 256 - 1);
                TIM_SetCompare1(TIM3, 127);
                TIM_SetCompare2(TIM3, 0);
                TIM_SetCompare3(TIM3, 0);
                TIM_SetCompare4(TIM3, 127);
                vTaskDelay(1200);
				//reset_Wheels();
				//vTaskDelay(100);
		}
        else if(/*data_sending != 1 && */ADC1ConvertedVoltage[0] <= 1500 && 2305 - ADC1ConvertedVoltage[0] > 2362 - ADC1ConvertedVoltage[1] && 2305 - ADC1ConvertedVoltage[0] > ADC1ConvertedVoltage[1] - 2362){  //right(1500~0)
        	/*if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_RIGHT;
            }*/
                //sprintf(buff, "right");
                //Usart3_Printf(buff); // send string to USART3
                TIM_SetCompare1(TIM1, 256 - 1);
				TIM_SetCompare2(TIM1, 256 - 1);
                TIM_SetCompare1(TIM3, 0);
                TIM_SetCompare2(TIM3, 127);
                TIM_SetCompare3(TIM3, 127);
                TIM_SetCompare4(TIM3, 0);
                vTaskDelay(1200);
				//reset_Wheels();
				//vTaskDelay(100);
		}/*
		else if(((ADC1ConvertedVoltage[0] <= 2200 && ADC1ConvertedVoltage[0] >= 1975) || (ADC1ConvertedVoltage[0] >= 2350 && ADC1ConvertedVoltage[0] <= 2555)) || 
	            ((ADC1ConvertedVoltage[1] <= 2335 && ADC1ConvertedVoltage[1] >= 2239) || (ADC1ConvertedVoltage[1] >= 2380 && ADC1ConvertedVoltage[1] <= 2480))){
			tremor_count ++;
		    if(tremor_count > 20){
		    	char w;
		    	USART_SendData(USARTy, w); //send a warning of tremor to android
		    	tremor_count = 0;
		    }
		} */
		else{
		}
 }
}