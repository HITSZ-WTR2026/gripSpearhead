#include "app/grip_spearhead.h"
#include "drivers/DJI.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "tim.h"
#include "interfaces/chassis_if.h"
#include "math.h"

PWM_t gripServo_PWM;
PWM_t helpServo_PWM;
Grip_t gripStruct = {
    .gripAngle = GRIPANGLE_PUT,
    .helpState = FREESTATE,
    .turnoverState = TURNOVER_INIT
};

DJI_t dji;
Motor_PosCtrl_t pos_dji;

Chassis_t chassis;

void Servo_Init()
{
    gripServo_PWM.htim = GRIP_TIMMER;
    gripServo_PWM.channel = GRIP_CHANNEL;
    helpServo_PWM.htim = GRIP_TIMMER;
    helpServo_PWM.channel = HELP_CHANNEL;
    PWM_Start(&gripServo_PWM);
    PWM_Start(&helpServo_PWM);
}

void TIM_Callback(TIM_HandleTypeDef* htim)
{
    Motor_PosCtrlUpdate(&pos_dji);
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
}

void DJI_Control_Init()
{
    DJI_CAN_FilterInit(&hcan1, 0);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);

    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    DJI_Init(&dji, (DJI_Config_t){
                        .auto_zero  = false,      //< 是否在启动时自动清零角度
                        .hcan       = &hcan1,     //< 电机挂载在的 CAN 句柄
                        .motor_type = M3508_C620, //< 电机类型
                        .id1        = 1,          //< 电调 ID (1~8)
                    });

    Motor_PosCtrl_Init(&pos_dji, //
                        (Motor_PosCtrlConfig_t){
                            .motor_type   = MOTOR_TYPE_DJI, //< 电机类型
                            .motor        = &dji,           //< 控制的电机
                            .velocity_pid = (MotorPID_Config_t){
                                .Kp             = 12.0f,  //<
                                .Ki             = 0.20f,  //<
                                .Kd             = 5.00f,  //<
                                .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                            },
                            .position_pid = (MotorPID_Config_t){
                                .Kp             = 80.0f,  //<
                                .Ki             = 1.00f,  //<
                                .Kd             = 0.00f,  //<
                                .abs_output_max = 2000.0f //< 限速，这是外环对内环的输出限幅
                            },
                            .pos_vel_freq_ratio = 1, //< 内外环频率比（外环的频率可能需要比内环低）
                        });

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

void grip_init()
{
    Servo_Init();
    DJI_Control_Init();
}

int gripSpearhead(Grip_t *gripStruct, PWM_t *gripServo_PWM, float gripState)
{
    if (gripState > GRIPANGLE_GRIP){
        gripStruct->gripAngle = GRIPANGLE_GRIP;
    }
    else {
        gripStruct->gripAngle = gripState;
    }
    PWM_SetDutyCircle(gripServo_PWM, gripStruct->gripAngle);
    return 0;
}

int gripSpearHelp(Grip_t *gripStruct, PWM_t *helpServo_PWM, float helpAngle)
{
    if (helpAngle > PLAMSTATE){
        gripStruct->helpState = PLAMSTATE;
    }
    else {
        gripStruct->helpState = helpAngle;
    }
    PWM_SetDutyCircle(helpServo_PWM, gripStruct->helpState);
    return 0;
}

int gripSpearTurn(Grip_t *gripStruct, Motor_PosCtrl_t *hposCtrl, float turnAngle)
{
    if (turnAngle == TURNOVER_INIT){
        gripStruct->turnoverState = TURNOVER_INIT;
        hposCtrl->position = TURNOVER_INIT;
    }
    else if (turnAngle == TURNOVER_MATCH){
        gripStruct->turnoverState = TURNOVER_MATCH;
        hposCtrl->position = TURNOVER_MATCH;
    }
    else{
        return -1;
    }
    return 0;
}

/* 夹取端头的任务函数 */
void vGripSpearheadTask(void *pvParameters)
{
    grip_init();
    float relativePositionCar[2] = {0};     //< 某一物体相对R2的二维坐标
    float relativePositionHead[3] = {0};    //< 目标端头的相对位置数据
    uint8_t spearheadFlag = 1;          //< 要夹取的端头标志，0已夹取，1-6优先级依次递减的端头(掌左->掌右->拳左->拳右->矛尖左->矛尖右)
    uint8_t findShelfFlag = 0;          //< 是否寻找到架子的标志位
    for ( ;; )
    {
        /* 1. 移动转圈寻找摆端头的架子，并移动到其附近 */ //< 逆时针转圈寻找
        do{ //< 转圈寻找
            Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) { 
                .posture = {
                    .x   = 0.0f,
                    .y   = 0.0f,
                    .yaw = 90.0f,
                },
                .speed  = 0.25f,
                .omega  = 30.0f,
            });
            osDelay(500);  //< 留给视觉一定的识别时间
            /* findShelfFlag = isFindShelf() */ //< 上位机传来是否找到架子
        } while (findShelfFlag == 0);

        do{ //< 移动
            /* relativePositionCar[2] = getRelativeShelfPos() */ //< 获取架子与R2的相对位置信息 relativePositionCar[2], [0]前后相对位置，[1]左右相对位置
            Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) {    
                .posture = {
                    .x   = relativePositionCar[1],
                    .y   = relativePositionCar[0],
                    .yaw = 0.0f,
                },
                .speed  = 0.25f,
                .omega  = 30.0f,
            });
            osDelay(200);
        } while ( (fabs(relativePositionCar[0]) < POS_ERRORMAX) && (fabs(relativePositionCar[1]) < POS_ERRORMAX) );

        /* 2. 明确要夹取的端头，并夹取 */
        while (spearheadFlag != 0)
        {
            /* findSpearhead( spearheadFlag ) */ //< 锁定要夹取的端头函数，【问题：是纯视觉锁定还是先底盘移动到目标一定范围内再视觉精准定位】
            
            if ( 0 /* isSpearheadEmple(spearheadFlag) */)   //< 判断目标端头位置是否为空，挂起等待视觉传来信息
            {
                spearheadFlag++;
            }
            else if ( 0 /* isOpponentGripSame(spearheadFlag) */) //< 判断对方是否夹取同一端头，挂起等待视觉传来信息
            {
                spearheadFlag++; //< 放弃当前端头，夹取下一个优先级的端头
            }
            else
            {
                /* 根据视觉获得的相对位置移动夹爪 */ //< 【夹爪上升高度应该可以写死】
                do{
                    //< 【如何获取上位机传来的信息】
                    /* relativePositionHead[3] = getRelativeHeadPos() */ //< 获取当前夹爪与目标端头的相对位置信息 relativePosition[3], [0]前后相对位置(目标端头相对夹爪靠前为正)，[1]左右相对位置(目标端头相对夹爪偏右为正)，[2]上下相对位置(目标端头相对夹爪偏上为正)
                    Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) {    //< 根据相对位置移动底盘
                        .posture = {
                        .x   = relativePositionHead[1],
                        .y   = relativePositionHead[0],
                        .yaw = 0.0f,
                        },
                        .speed  = 0.25f,
                        .omega  = 30.0f,
                    });
                    /* upGripper(relativePosition[2])*/ //< 根据相对位置夹爪上下移动【要与越障升降共用函数】【待定】
                } while ( (relativePositionHead[0] < MAXERROR) && (relativePositionHead[1] < MAXERROR) && (relativePositionHead[2] < MAXERROR));
                /* 夹取目标端头 */
                gripSpearhead(&gripStruct, &gripServo_PWM, GRIPANGLE_GRIP);
                spearheadFlag = 0;
            }
            if (spearheadFlag > 6) 
            {
                break;
            }
        }

        /* 3. 辅助机构辅助 */
        if ((spearheadFlag + 1) / 2 == 1)
        {
            gripSpearHelp(&gripStruct, &helpServo_PWM, PLAMSTATE);
        }
        else if ((spearheadFlag + 1) / 2 == 2)
        {
            gripSpearHelp(&gripStruct, &helpServo_PWM, FISTSTATE);
        }
        else if ((spearheadFlag + 1) / 2 == 3)
        {
            gripSpearHelp(&gripStruct, &helpServo_PWM, SPEARSTATE);
        }
        else
        {
            /* 所有端头已被拿走 */
        }

        /* 4. 翻转夹爪 */
        gripSpearTurn(&gripStruct, &pos_dji, TURNOVER_MATCH);
        
        /* 5. 转身 */
        Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) {    //< 具体数值待定
            .posture = {
                .x   = 0.0f,
                .y   = 0.0f,
                .yaw = 180.0f,
            },
            .speed  = 0.25f,
            .omega  = 30.0f,
        });

        /* 6. 等待R1完成武器对接 */
        /* isFinishDock() */ //< 挂起等待视觉判断R1完成武器对接的信息传来

        /* 7. 判断R1已离开武馆区 */ //< 【这里也可以选择跟随R1离开武馆区】
        /* isLeave_R1() */ //< 挂起等待视觉判断R1已离开的信息传来

        /* 8. 离开武馆区 */
        do{ //< 移动
            /* relativePositionCar[2] = getRelativeExitPos() */ //< 获取出口与R2的相对位置信息 relativePositionCar[2], [0]前后相对位置，[1]左右相对位置
            Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) {    
                .posture = {
                    .x   = relativePositionCar[1],
                    .y   = relativePositionCar[0],
                    .yaw = 0.0f,
                },
                .speed  = 0.25f,
                .omega  = 30.0f,
            });
            osDelay(200);
        } while ( (fabs(relativePositionCar[0]) < POS_ERRORMAX) && (fabs(relativePositionCar[1]) < POS_ERRORMAX) );
        //< 转身离开
        Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) {    
            .posture = {
                .x   = 0.0f,
                .y   = 0.0f,
                .yaw = 90.0f,
            },
            .speed  = 0.25f,
            .omega  = 30.0f,
        });
    }
}
