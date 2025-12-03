#include "app/grip_spearhead.h"
#include "drivers/DJI.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "tim.h"
#include "interfaces/chassis_if.h"
#include "math.h"
#include "cmsis_os2.h"
#include "usart.h"

// 全局消息队列：用于传递上位机命令（假设每条命令 ≤ 64 字节）
osMessageQueueId_t g_cmdQueue = NULL;
#define CMD_MAX_LEN 64

PWM_t gripServo_PWM;
PWM_t helpServo_PWM;
Grip_t gripStruct;
GripFeedback feedbackDatas;

DJI_t dji;
Motor_PosCtrl_t pos_dji;

Chassis_t chassis;      ///< 接入整体代码后不再需要写，只是为了不报错加的

/*  第一个字节为“0x00”，表示返回的数据是各种是否情况的状态返回；
    第一个字节为“0x11”，表示返回的数据是目标端头与夹爪的相对位置；
    第一个字节为“0x22”，表示返回的数据是R2与武馆出口的相对位置； */
// 使用轮询方式（简单，适合低速）
void UartReceiverTask(void *argument) {
    uint8_t byte;
    uint8_t buffer[CMD_MAX_LEN];
    uint8_t idx = 0;

    for (;;) {
        // 轮询接收一个字节（1ms 超时）
        if (HAL_UART_Receive(&huart1, &byte, 1, 1) == HAL_OK) {
            if (byte == '\n') {
                // 帧结束
                if (idx > 0) {
                    // 将完整命令拷贝到队列（注意：CMSIS 队列拷贝整个对象）
                    osMessageQueuePut(g_cmdQueue, buffer, 0, 0);
                    idx = 0;
                }
            } else if (idx < CMD_MAX_LEN - 1) {
                buffer[idx++] = byte;
            }
        }
        osDelay(1); // 避免忙等
    }
}

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
    gripStruct.gripAngle = GRIPANGLE_PUT;
    gripStruct.helpState = FREESTATE;
    gripStruct.turnoverState = TURNOVER_INIT;
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

void movePosition(float postureX, float postureY)
{
    Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) { 
        .posture = {
            .x   = postureX,
            .y   = postureY,
            .yaw = 0.0f,
        },
        .speed  = 0.25f,
        .omega  = 30.0f,
    });
}

void moveTurn(float postureYaw)
{
    Chassis_SetTargetPostureInBody(&chassis,&(Chassis_PostureTarget_t) { 
        .posture = {
            .x   = 0.0f,
            .y   = 0.0f,
            .yaw = postureYaw,
        },
        .speed  = 0.25f,
        .omega  = 30.0f,
    });
}

/* 夹取端头的任务函数 */
void vGripSpearheadTask(void *pvParameters)
{
    uint8_t cmdBuffer[CMD_MAX_LEN];
    uint8_t headGrippedNum = 0;
    uint8_t response[3] = {0xA0,0x00,0xB0};
    grip_init();
    for ( ;; )
    {
        /* 1. 移动到摆端头的架子附近 */ //< 【初步移动到架子前】
        movePosition(SHELF_POSITION_X, SHELF_POSITION_Y);
        osDelay(1000);      //< 给移动留一定时间
        /* upGripper(GRIP_HEIGHT)*/ //< 根据相对位置夹爪上下移动【要与越障升降共用函数】【待定】

        /* 2. 明确要夹取的端头，并夹取 */
        while (headGrippedNum < NEEDHEADNUM)
        {
            response[1] = 0x10 + feedbackDatas.spearheadFlag;
            HAL_UART_Transmit(&huart1, (uint8_t*)response, 1, HAL_MAX_DELAY);       //< 传给上位机消息，要求去返回目标端头的状态或位置信息
            do{
                //< 视觉锁定目标端头，并传来数据
                if (osMessageQueueGet(g_cmdQueue, cmdBuffer, NULL, osWaitForever) == osOK) {
                    if (cmdBuffer[0] == 0x00){                              //< 抢同一端头或目标端头已被拿走的情况
                        if (cmdBuffer[1] == 0x00 || cmdBuffer[1] == 0xff){
                            feedbackDatas.spearheadFlag++;
                            if (feedbackDatas.spearheadFlag > 6) {
                                break;
                            }
                        }
                    }
                    else if (cmdBuffer[0] == 0x11){                         //< 无特殊情况，传回夹爪与目标端头的相对位置
                        feedbackDatas.postureX = cmdBuffer[1];
                        feedbackDatas.postureY = cmdBuffer[2];
                        movePosition(feedbackDatas.postureX, feedbackDatas.postureY);
                    }
                }
            } while ( (cmdBuffer[0] == 0x00) || ((feedbackDatas.postureX > MAXERROR) && (feedbackDatas.postureY > MAXERROR)));
            /* 夹取目标端头 */
            gripSpearhead(&gripStruct, &gripServo_PWM, GRIPANGLE_GRIP);

            /* 3. 辅助机构辅助 */
            if ((feedbackDatas.spearheadFlag + 1) / 2 == 1)
            {
                gripSpearHelp(&gripStruct, &helpServo_PWM, PLAMSTATE);
            }
            else if ((feedbackDatas.spearheadFlag + 1) / 2 == 2)
            {
                gripSpearHelp(&gripStruct, &helpServo_PWM, FISTSTATE);
            }
            else if ((feedbackDatas.spearheadFlag + 1) / 2 == 3)
            {
                gripSpearHelp(&gripStruct, &helpServo_PWM, SPEARSTATE);
            }
            else
            {
                /* 所有端头已被拿走 */
            }

            /* 4. 翻转夹爪 */
            gripSpearTurn(&gripStruct, &pos_dji, TURNOVER_MATCH);
            /* upGripper(0)*/ //< 夹爪下降回最低处【要与越障升降共用函数】【待定】
        
            /* 5. 转身 */
            moveTurn(180.0);    //< 【具体数值待定】

            /* 6. 等待R1完成武器对接 */
            do{
                response[1] = 0xAA;
                HAL_UART_Transmit(&huart1, (uint8_t*)response, 1, HAL_MAX_DELAY);           //< 传给上位机消息，要求去判断端头是否对接完成
                if (osMessageQueueGet(g_cmdQueue, cmdBuffer, NULL, osWaitForever) == osOK){
                    if (cmdBuffer[0] == 0x00){
                        if (cmdBuffer[1] == 0x11){
                            feedbackDatas.isFinishDockFlag = 1;
                        }
                        else {
                            feedbackDatas.isFinishDockFlag = 0;
                        }
                    }
                }
                osDelay(100);
            } while (feedbackDatas.isFinishDockFlag == 0);
            
            /* 完成了一个端头的夹取和对接 */
            headGrippedNum++;
            if (headGrippedNum > 1){
                moveTurn(180.0);        ///< 转身继续夹取
            }
        }

        /* 7. 判断R1已离开武馆区 */ //< 【这里也可以选择跟随R1离开武馆区】
        do{
            //< 视觉判断R1是否已离开武馆区
            response[1] = 0xBB;
            HAL_UART_Transmit(&huart1, (uint8_t*)response, 1, HAL_MAX_DELAY);       //< 传给上位机消息，要求去判断R1是否已离开武馆区
            if (osMessageQueueGet(g_cmdQueue, cmdBuffer, NULL, osWaitForever) == osOK){
                if (cmdBuffer[0] == 0x00){
                    if (cmdBuffer[1] == 0x22){
                        feedbackDatas.isR1LeaveFlag = 1;
                    }
                    else {
                        feedbackDatas.isR1LeaveFlag = 0;
                    }
                }
            }
            osDelay(100);
        } while (feedbackDatas.isR1LeaveFlag == 0);

        /* 8. 离开武馆区 */
        do{ //< 移动
            response[1] = 0xCC;
            HAL_UART_Transmit(&huart1, (uint8_t*)response, 1, HAL_MAX_DELAY);       //< 传给上位机消息，要求返回武馆区出口的位置信息
            if (osMessageQueueGet(g_cmdQueue, cmdBuffer, NULL, osWaitForever) == osOK){
                if (cmdBuffer[0] == 0x22){
                    feedbackDatas.postureX = cmdBuffer[1];
                    feedbackDatas.postureY = cmdBuffer[2];
                }
            }
            movePosition(feedbackDatas.postureX, feedbackDatas.postureY);
            osDelay(100);
        } while ( (fabs(feedbackDatas.postureX) < POS_ERRORMAX) && (fabs(feedbackDatas.postureX) < POS_ERRORMAX) );
        //< 转身离开
        moveTurn(90.0);     //< 【具体数值待定】
    }
}
