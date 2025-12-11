#include "Power_distribution.h"

//float current_MA = 0;
//float current_MB = 0;
//float current_MC = 0;
//float current_MD = 0;

//float power_MA = 0;
//float power_MB = 0;
//float power_MC = 0;
//float power_MD = 0;
//const float Kt = 0.3f; // 转矩常数

//// 将原始电流值转换为实际电流值(A)
//float ConvertRawCurrentToAmps(int16_t raw_current)
//{
//    // 线性映射: raw_current ∈ [-16384, 16384] -> amps ∈ [-20.0, 20.0]
//    return (raw_current - CURRENT_RAW_MIN) * 
//           (CURRENT_ACTUAL_MAX - CURRENT_ACTUAL_MIN) / 
//           (CURRENT_RAW_MAX - CURRENT_RAW_MIN) + 
//           CURRENT_ACTUAL_MIN;
//}

//float Power_PID_Calculate(float setpoint, float actual)
//{
//    power_ctrl.error = setpoint - actual;
//    
//    power_ctrl.integral += power_ctrl.error;
//    
//    if (power_ctrl.integral > POWER_PID_MAX_OUTPUT / POWER_PID_KI)
//    {
//        power_ctrl.integral = POWER_PID_MAX_OUTPUT / POWER_PID_KI;
//    } 
//    else if (power_ctrl.integral < POWER_PID_MIN_OUTPUT / POWER_PID_KI)
//    {
//        power_ctrl.integral = POWER_PID_MIN_OUTPUT / POWER_PID_KI;
//    }
//    
//    power_ctrl.derivative = power_ctrl.error - power_ctrl.last_error;
//    power_ctrl.last_error = power_ctrl.error;
//    
//    float output = POWER_PID_KP * power_ctrl.error + 
//                   POWER_PID_KI * power_ctrl.integral + 
//                   POWER_PID_KD * power_ctrl.derivative;
//   
//    if (output > POWER_PID_MAX_OUTPUT)
//    {
//        output = POWER_PID_MAX_OUTPUT;
//    } 
//    else if (output < POWER_PID_MIN_OUTPUT)
//    {
//        output = POWER_PID_MIN_OUTPUT;
//    }
//    
//    return output;
//}

//void Power_all_Calculate(void)
//{
//    current_MA = ConvertRawCurrentToAmps(set_currentMA); //单位A
//    current_MB = ConvertRawCurrentToAmps(set_currentMB);
//    current_MC = ConvertRawCurrentToAmps(set_currentMC);
//    current_MD = ConvertRawCurrentToAmps(set_currentMD);
//    
//    power_MA = fabsf(current_MA) * Kt * fabsf((float)Wheel_M3508Handle[0].Basic_Data->speed_rpm) * POWER_ESTIMATE_K;
//    power_MB = fabsf(current_MB) * Kt * fabsf((float)Wheel_M3508Handle[1].Basic_Data->speed_rpm) * POWER_ESTIMATE_K;
//    power_MC = fabsf(current_MC) * Kt * fabsf((float)Wheel_M3508Handle[2].Basic_Data->speed_rpm) * POWER_ESTIMATE_K;
//    power_MD = fabsf(current_MD) * Kt * fabsf((float)Wheel_M3508Handle[3].Basic_Data->speed_rpm) * POWER_ESTIMATE_K;
//    
//    power_ctrl.total_power = power_MA + power_MB + power_MC + power_MD;
//}


//全向轮
MotorPowerModel_t power_model = {
    .k1 = 1.53e-07f,              // 需要根据Matlab拟合结果填写
    .k2 = 1.553e-07f,             // 需要根据Matlab拟合结果填写
    .a = 8.081f,                  // 需要根据Matlab拟合结果填写
    .Ct = 1.996e-6f,              // 固定值
    .Kt = 0.01562,                //0.3*(187/3591)减速比
    .max_power = 50.0f            //最大功率限制
};


//麦轮
//MotorPowerModel_t power_model = {
//    .k1 = 1.83e-07f,              // 需要根据Matlab拟合结果填写
//    .k2 = 1.853e-07f,             // 需要根据Matlab拟合结果填写
//    .a = 5.081f,                  // 需要根据Matlab拟合结果填写  (传输损耗)
//    .Ct = 1.96e-5f,              // 固定值 Kt  * (20/16384)
//    .Kt = 0.01562,
//    .max_power = 50.0f 
//};
// 功率计算函数
float CalculateMotorPower(int16_t I_cmd, int16_t speed_rpm)
{   
    float mechanical_power = power_model.Ct * I_cmd * speed_rpm;
    
    float speed_term = power_model.k1 * speed_rpm * speed_rpm;
    
    float current_term = power_model.k2 * I_cmd * I_cmd * power_model.Kt * power_model.Kt;
    
    return mechanical_power + speed_term + current_term + power_model.a;
}

// 电流求解函数（根据目标功率和当前转速求解应发送的I_cmd）
int16_t SolveForTargetCurrent(float target_power, int16_t speed_rpm, int16_t original_I_cmd)
{
    float k1 = power_model.k1;
    float k2 = power_model.k2;
    float a = power_model.a;
    float Ct = power_model.Ct;
    float solution = 0.0f;
    
    //k2*Kt^2*(20/16384)^2*I_cmd^2 + Ct*ω*I_cmd + (k1*ω^2 + a - target_power) = 0
    float A = k2 * power_model.Kt * power_model.Kt * 0.001220703125f * 0.001220703125f; //0.001220703125 = 20 / 16384
    float B = Ct * speed_rpm;
    float C = k1 * speed_rpm * speed_rpm + a - target_power;
    
    // 鲁棒性求解：处理A接近0的情况
    if (fabsf(A) < 1.0e-6f)
    {
        if (fabsf(B) > 1.0e-6f)
        {
            solution = -C / B;
        } 
        else
        {
            solution = 0.0f; // 无解时返回0
        }
    }
    else
    {
        float discriminant = B * B - 4 * A * C;
        
        if (discriminant < 0)
        {
            // 无实根：返回顶点位置，使方程值最小化
            solution = -B / (2 * A);
        } 
        else
        {
            // 计算两个根
            float sqrt_disc = sqrtf(discriminant);
            float sol1 = (-B + sqrt_disc) / (2 * A);
            float sol2 = (-B - sqrt_disc) / (2 * A);
            
            // 选择与原始电流更接近的解，以保持平滑性
            if (fabsf(sol1 - original_I_cmd) < fabsf(sol2 - original_I_cmd))
            {
                solution = sol1;
            } 
            else 
            {
                solution = sol2;
            }
        }
    }
    
    return (int16_t)solution;
}

// 功率分配主函数
void PowerDistribution(int16_t* currents, int16_t* speeds)
{
    float total_power = 0.0f;
    float individual_powers[4] = {0};
    static float filtered_power_scale = 1.0f;
    static int16_t previous_currents[4] = {0};     // 保存上一个周期的电流值，用于斜率限制
    float lpf_alpha = 0.1f;                        //值越小越平滑但响应变慢
    
    // 计算每个电机的功率和总功率
    for (int i = 0; i < 4; i++)
    {
        individual_powers[i] = CalculateMotorPower(currents[i], speeds[i]); // 假设此函数已实现
        total_power += individual_powers[i];
    }
    
    // 检查是否超过总功率限制
    if (total_power <= power_model.max_power)
    {
        return; // 不需要功率分配
    }
    
    // 计算功率缩放系数
    float power_scale = power_model.max_power / total_power;  //计算得到总功率 / 最大限制功率
    
    // 应用低通滤波平滑输出功率
    filtered_power_scale = filtered_power_scale * (1.0f - lpf_alpha) + power_scale * lpf_alpha;
    
    // 对每个电机进行功率再分配
    for (int i = 0; i < 4; i++)
    {
        // 跳过输出负功的电机（不消耗功率）
        if (individual_powers[i] <= 0)
        {
            continue;
        }
        
        // 计算目标功率
        float target_power = individual_powers[i] * filtered_power_scale;
        
        // 求解新电流指令
        int16_t new_current = SolveForTargetCurrent(target_power, speeds[i], currents[i]); //currents[i]得到平滑曲线
        
        // 应用斜率限制：防止电流突变，增强平滑性
        int16_t max_change = 100;//单个周期最大电流变化量，根据电机性能调整（例如100mA）
        if (new_current > previous_currents[i] + max_change)
        {
            currents[i] = previous_currents[i] + max_change;
        }
        else if (new_current < previous_currents[i] - max_change)
        {
            currents[i] = previous_currents[i] - max_change;
        }
        else
        {
            currents[i] = new_current;
        }
        
        // 更新上一个周期电流值
        previous_currents[i] = currents[i];
    }
}
