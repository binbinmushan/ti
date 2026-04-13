/**
 * @file    pid.c
 * @brief   STM32 通用 PID 控制器实现（位置式 & 增量式）
 */

#include "pid.h"

/* ------------------------------------------------------------------ */
/*  内部辅助函数                                                        */
/* ------------------------------------------------------------------ */

static float clamp(float value, float min, float max)
{
    /* 若 min > max 则不限幅，直接返回原值，避免未定义行为 */
    if (min > max) return value;
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* ------------------------------------------------------------------ */
/*  公共接口实现                                                        */
/* ------------------------------------------------------------------ */

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_max, float out_min,
              float integral_max, float integral_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->out_max      = out_max;
    pid->out_min      = out_min;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->setpoint        = 0.0f;
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output          = 0.0f;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Reset(PID_t *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output          = 0.0f;
}

/* ------------------------------------------------------------------ */
/*  位置式 PID                                                          */
/*                                                                      */
/*  u(k) = Kp * e(k)                                                   */
/*       + Ki * ∑e(k)          （积分限幅防饱和）                       */
/*       + Kd * [e(k) - e(k-1)]                                        */
/* ------------------------------------------------------------------ */

float PID_Compute(PID_t *pid, float measured)
{
    float error = pid->setpoint - measured;

    /* 积分（带限幅防饱和） */
    pid->integral += error;
    pid->integral  = clamp(pid->integral, pid->integral_min, pid->integral_max);

    /* 微分 */
    float derivative = error - pid->prev_error;

    /* 输出 */
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    output = clamp(output, pid->out_min, pid->out_max);

    pid->prev_error = error;

    return output;
}

/* ------------------------------------------------------------------ */
/*  增量式 PID                                                          */
/*                                                                      */
/*  Δu(k) = Kp * [e(k) - e(k-1)]                                      */
/*         + Ki *  e(k)                                                 */
/*         + Kd * [e(k) - 2*e(k-1) + e(k-2)]                          */
/*  u(k) = u(k-1) + Δu(k)                                              */
/* ------------------------------------------------------------------ */

float PID_ComputeIncremental(PID_t *pid, float measured)
{
    float error = pid->setpoint - measured;

    float delta = pid->Kp * (error - pid->prev_error)
                + pid->Ki *  error
                + pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error);

    pid->output += delta;
    pid->output  = clamp(pid->output, pid->out_min, pid->out_max);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error      = error;

    return pid->output;
}
