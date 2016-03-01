/*=============================================================================
 *
 * @file     : EPW_behavior.h
 * @author        : JackABK
 * @data       : 2014/2/3
 * @brief   : car_behavior.c header file
 *
 *============================================================================*/
#ifndef __EPW_BEHAVIOR_H__
#define __EPW_BEHAVIOR_H__


/*=================Re-define the all by pins=========================*/
/****Joystick****/
#define JOYSTICK_PORT                                             GPIOC
#define JOYSTICK_X_AXIS_PIN                                       GPIO_Pin_0
#define JOYSTICK_Y_AXIS_PIN                                       GPIO_Pin_1
//__IO uint16_t ADC1ConvertedVoltage[2];
int16_t ADC1ConvertedVoltage[2];

/*===============end of define  the all by pins========================*/

extern void parse_Joystick_dir();

#endif /* __CAR_BEHAVIOR_H__ */
