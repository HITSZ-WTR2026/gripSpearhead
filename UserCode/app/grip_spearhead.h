#ifndef GRIP_SPEARHEAD_H
#define GRIP_SPEARHEAD_H

#include "bsp\pwm.h"
#include "interfaces/motor_if.h"

/* 两舵机的PWM所用定时器和通道 */
#define GRIP_TIMMER (&htim9)
#define GRIP_CHANNEL TIM_CHANNEL_1
#define HELP_CHANNEL TIM_CHANNEL_2

/* 夹爪松与夹 */
#define GRIPANGLE_PUT   (0)     //< 夹爪松开，【具体角度待定】
#define GRIPANGLE_GRIP  (90)    //< 夹爪夹紧，【具体角度待定】
/* 夹爪翻转状态 */
#define TURNOVER_INIT   (0)     //< 夹爪翻转初态，【具体角度待定】
#define TURNOVER_MATCH  (180)   //< 夹爪翻转配对状态，【具体角度待定】
/* 夹爪辅助机构状态 */
#define FREESTATE (0)           //< 无辅助状态 【具体角度待定】
#define SPEARSTATE (10)         //< 辅助矛尖状态 【具体角度待定】
#define FISTSTATE (20)          //< 辅助拳状态 【具体角度待定】
#define PLAMSTATE (30)          //< 辅助掌状态 【具体角度待定】

#define MAXERROR (1)            //< 夹爪与目标端头的最大相对位置误差，即能保证正常夹取的最大误差
#define POS_ERRORMAX (10)       //< 移动位置最大相对误差

typedef struct grip
{
    float gripAngle;
    float helpState;
    float turnoverState;
} Grip_t;

void grip_init();
int gripSpearhead(Grip_t *gripStruct, PWM_t *gripServo_PWM, float gripState);
int gripSpearHelp(Grip_t *gripStruct, PWM_t *helpServo_PWM, float helpAngle);
int gripSpearTurn(Grip_t *gripStruct, Motor_PosCtrl_t *hposCtrl, float turnAngle);

#endif
