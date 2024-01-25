#include "odrive.h"
#include "can.h"

union_32 union_32f;
union_16 union_16f;
FeedBack feedback;
SetState Setstate;
SetMode Setmode;
Feedback GIM6010[9];
/**
 * ���Ŀ���ٶ�ֵ
 * @param _hcan
 * @param Input_Vel
 * @param Torque
 * @param stdid
 */
void Odrive_Axis_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque,uint32_t stdid)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

	union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * ���Ŀ��Ƕ�ֵ
 * @param _hcan
 * @param Input_Pos
 * @param Vel_FF
 * @param Torque_FF
 * @param stdid
 */
void Odrive_Axis_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF,uint32_t stdid)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * �������õ����״̬�����а������У�IDLE������ջ�ģʽ��
 * @param _hcan
 * @param State
 * @param stdid
 */
void Odrive_Set_State(CAN_HandleTypeDef *_hcan, uint32_t State,uint32_t stdid)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    Setstate.data_id = State;

    TxData[0] = Setstate.data_8[0];
    TxData[1] = Setstate.data_8[1];
    TxData[2] = Setstate.data_8[2];
    TxData[3] = Setstate.data_8[3];
    TxData[4] = Setstate.data_8[4];
    TxData[5] = Setstate.data_8[5];
    TxData[6] = Setstate.data_8[6];
    TxData[7] = Setstate.data_8[7];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * �������õ���Ŀ���ģʽ
 * @param _hcan
 * @param ContorlMode
 * @param InputMode
 * @param stdid
 */
void Odrive_Set_ControlMode(CAN_HandleTypeDef *_hcan, uint32_t ContorlMode,uint32_t InputMode,uint32_t stdid)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    Setmode.data_c = ContorlMode;
    TxData[0] = Setmode.data_8[0];
    TxData[1] = Setmode.data_8[1];
    TxData[2] = Setmode.data_8[2];
    TxData[3] = Setmode.data_8[3];

    Setmode.data_i = InputMode;
    TxData[4] = Setmode.data_8[0];
    TxData[5] = Setmode.data_8[1];
    TxData[6] = Setmode.data_8[2];
    TxData[7] = Setmode.data_8[3];


    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * ������ȡ������������ٶ���λ�õ�ֵ�����ݿ���Ϊ0
 * @param _hcan
 * @param Pos
 * @param Vel
 * @param stdid
 */
void Odrive_Encoder_feedback(CAN_HandleTypeDef *_hcan, float Pos,float Vel,uint32_t stdid)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    feedback.data_pos = Pos;
    TxData[0] = Setmode.data_8[0];
    TxData[1] = Setmode.data_8[1];
    TxData[2] = Setmode.data_8[2];
    TxData[3] = Setmode.data_8[3];

    feedback.data_vel = Vel;
    TxData[4] = Setmode.data_8[0];
    TxData[5] = Setmode.data_8[1];
    TxData[6] = Setmode.data_8[2];
    TxData[7] = Setmode.data_8[3];


    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * ��Ŀ��id�����ͨ��ģʽ��CANתΪUSB
 * @param _hcan
 * @param stdid
 */
void Odrive_CAN_to_USB(CAN_HandleTypeDef *_hcan,uint32_t stdid)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = stdid;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    feedback.data_pos = 0;
    TxData[0] = Setmode.data_8[0];
    TxData[1] = Setmode.data_8[1];
    TxData[2] = Setmode.data_8[2];
    TxData[3] = Setmode.data_8[3];

    feedback.data_vel = 0;
    TxData[4] = Setmode.data_8[0];
    TxData[5] = Setmode.data_8[1];
    TxData[6] = Setmode.data_8[2];
    TxData[7] = Setmode.data_8[3];


    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * ���ͱ�����֮��Ľ��������������ȡ�����ǰ���ٶ���Ƕ�
 * @param ptr �������ĵĽṹ��
 * @param data ���յ��Ĵ�ŷ������ĵ�����
 */
void Odrive_feedback_record(Feedback *ptr,uint8_t *data)
{
    ptr->Pos = (float ) ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
    ptr->Vel = (float ) ((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
}
/**
 * ��λ�û�����������޷�
 * @param Limit_Speed
 */
void AllMotorSpeedLimit(int16_t Limit_Speed)
{
    for(int i=1;i < 9;i++)
    {
        AngleLoop[i].Output_limit = Limit_Speed;
    }
}

void Odrive_val_output(uint8_t id)
{
    switch(id)
    {
        case 1:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[1].Out_put,0.0f,Set_Axis1_Set_Input_Vel);
            osDelay(1);
            break;
        case 2:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[2].Out_put,0.0f,Set_Axis2_Set_Input_Vel);
            osDelay(1);
            break;
        case 3:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[3].Out_put,0.0f,Set_Axis3_Set_Input_Vel);
            osDelay(1);
            break;
        case 4:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[4].Out_put,0.0f,Set_Axis4_Set_Input_Vel);
            osDelay(1);
            break;
        case 5:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[5].Out_put,0.0f,Set_Axis5_Set_Input_Vel);
            osDelay(1);
            break;
        case 6:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[6].Out_put,0.0f,Set_Axis6_Set_Input_Vel);
            osDelay(1);
            break;
        case 7:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[7].Out_put,0.0f,Set_Axis7_Set_Input_Vel);
            osDelay(1);
            break;
        case 8:
            Odrive_Axis_Set_Input_Vel(&hcan1,AngleLoop[8].Out_put,0.0f,Set_Axis8_Set_Input_Vel);
            osDelay(1);
            break;
        default:
            break;
    }
}

void Odrive_Postion_output(uint8_t id)
{
    switch(id)
    {
        case 1:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[1],1,0,Set_Axis1_Set_Input_Pos);
            osDelay(1);
            break;
        case 2:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[2],1,0,Set_Axis2_Set_Input_Pos);
            osDelay(1);
            break;
        case 3:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[3],1,0,Set_Axis3_Set_Input_Pos);
            osDelay(1);
            break;
        case 4:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[4],1,0,Set_Axis4_Set_Input_Pos);
            osDelay(1);
            break;
        case 5:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[5],1,0,Set_Axis5_Set_Input_Pos);
            osDelay(1);
            break;
        case 6:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[6],1,0,Set_Axis6_Set_Input_Pos);
            osDelay(1);
            break;
        case 7:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[7],1,0,Set_Axis7_Set_Input_Pos);
            osDelay(1);
            break;
        case 8:
            Odrive_Axis_Set_Input_Position(&hcan1,AngleWant_MotorX[8],1,0,Set_Axis8_Set_Input_Pos);
            osDelay(1);
            break;
        default:
            break;
    }
}
/**
 * ���Ͱ˸�������ٶ����
 */
void AllMotor_valOutput(void)
{
    for(uint8_t Id = 1;Id < 9;Id ++)
    {
        Odrive_val_output(Id);
    }
}
/**
 * ���Ͱ˸������λ�����
 */
void AllMotor_PositionOutput(void)
{
    for(uint8_t Id = 1;Id < 9;Id ++)
    {
        Odrive_Postion_output(Id);
    }
}

void Motor_Init(void)
{
    /**
     * ���ÿ���ϵ���Ҫ�Ƚ���У׼���ܽ���ջ���û�кõĽ������
     * Ҳ�����Ҹ���ˣ�֮����ʱ�������Կɲ�����ֱ�ӽ���ջ�ģʽ
     */
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis1_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis2_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis3_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis4_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis5_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis6_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis7_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_MOTOR_CALIBRATION,Set_Axis8_Requested_State);
    osDelay(1);

    osDelay(6000); //���ȴ�6s���У׼���

    /**
     * ����ָ��ʹ�������ջ�����ģʽ
     */

    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis1_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis2_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis3_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis4_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis5_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis6_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis7_Requested_State);
    osDelay(1);
    Odrive_Set_State(&hcan1, AXIS_STATE_CLOSED_LOOP_CONTROL,Set_Axis8_Requested_State);
    osDelay(1);

}