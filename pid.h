/**
 * @file    pid.h
 * @brief   STM32 通用 PID 控制器（位置式 & 增量式）
 *
 * 使用方法：
 *  1. 定义一个 PID_t 结构体并调用 PID_Init() 初始化。
 *  2. 每个控制周期调用 PID_Compute()（位置式）或
 *     PID_ComputeIncremental()（增量式）。
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  数据结构                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    /* --- 整定参数 --- */
    float Kp;           /**< 比例系数 */
    float Ki;           /**< 积分系数 */
    float Kd;           /**< 微分系数 */

    /* --- 输出限幅 --- */
    float out_max;      /**< 输出上限 */
    float out_min;      /**< 输出下限 */

    /* --- 积分限幅（防积分饱和） --- */
    float integral_max; /**< 积分项上限 */
    float integral_min; /**< 积分项下限 */

    /* --- 内部状态 --- */
    float setpoint;     /**< 目标值（设定值） */
    float integral;     /**< 积分累计 */
    float prev_error;   /**< 上一次误差（用于微分） */
    float prev_prev_error; /**< 上上次误差（增量式专用） */
    float output;       /**< 上一次输出（增量式累计） */
} PID_t;

/* ------------------------------------------------------------------ */
/*  函数声明                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化 PID 控制器
 * @param  pid          PID 结构体指针
 * @param  Kp           比例系数
 * @param  Ki           积分系数
 * @param  Kd           微分系数
 * @param  out_max      输出上限
 * @param  out_min      输出下限
 * @param  integral_max 积分项上限（防饱和）
 * @param  integral_min 积分项下限（防饱和）
 */
void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_max, float out_min,
              float integral_max, float integral_min);

/**
 * @brief  设置目标值（设定值）
 * @param  pid      PID 结构体指针
 * @param  setpoint 新的目标值
 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

/**
 * @brief  重置 PID 内部状态（积分、误差、输出清零）
 * @param  pid  PID 结构体指针
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  位置式 PID 计算
 *
 *   output = Kp*e + Ki*∑e + Kd*(e - e_prev)
 *
 * @param  pid      PID 结构体指针
 * @param  measured 当前测量值
 * @return 控制输出（已限幅）
 */
float PID_Compute(PID_t *pid, float measured);

/**
 * @brief  增量式 PID 计算
 *
 *   Δu = Kp*(e - e_prev) + Ki*e + Kd*(e - 2*e_prev + e_prev2)
 *   output += Δu
 *
 * @param  pid      PID 结构体指针
 * @param  measured 当前测量值
 * @return 控制输出（已限幅）
 */
float PID_ComputeIncremental(PID_t *pid, float measured);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
