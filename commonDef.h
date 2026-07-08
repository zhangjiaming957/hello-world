/**
 * @file    commonDef.h
 * @author  Hecate
 * @brief   全局配置中心 (引脚映射 / PID参数 / 阈值 / 循迹差速表 / 数学宏)
 * @date    20260602
 */
#ifndef WAYTRACK_COMMON_DEF_H_
#define WAYTRACK_COMMON_DEF_H_

#include <stdint.h>
#include "hal/ledc_types.h"

/* ==================== 光电管(ADC) ==================== */
#define TRACK_CHANNEL_COUNT   5
#define TRACK_ADC_ATTEN       ADC_ATTEN_DB_12
#define TRACK_ADC_BITWIDTH    ADC_BITWIDTH_12

#define TRACK_PIN_1           27
#define TRACK_PIN_2           33
#define TRACK_PIN_3           32
#define TRACK_PIN_4           35
#define TRACK_PIN_5           34

/* 施密特阈值: raw 12bit计数(0~4095), 每通道独立可调
 * 双阈值迟滞, 越过上阈判为黑(1), 跌破下阈判为白(0) */
#define SCHMITT_U0             140.0f  /* ch0(GPIO27) 上阈值 */
#define SCHMITT_L0             100.0f   /* ch0(GPIO27) 下阈值 */
#define SCHMITT_U1             130.0f  /* ch1(GPIO33) 上阈值 */
#define SCHMITT_L1             100.0f   /* ch1(GPIO33) 下阈值 */
#define SCHMITT_U2             110.0f  /* ch2(GPIO32) 上阈值 */
#define SCHMITT_L2             80.0f   /* ch2(GPIO32) 下阈值 */
#define SCHMITT_U3             110.0f  /* ch3(GPIO35) 上阈值 */
#define SCHMITT_L3             80.0f   /* ch3(GPIO35) 下阈值 */
#define SCHMITT_U4             90.0f  /* ch4(GPIO34) 上阈值 */
#define SCHMITT_L4             60.0f   /* ch4(GPIO34) 下阈值 */

/* ==================== 电机(双路PWM, 非PWM脚输出高电平) ==================== */
/* 正转: IN1=PWM调速, IN2=HIGH | 反转: IN1=HIGH, IN2=PWM调速 */
#define MOTOR_A_IN1           15
#define MOTOR_A_IN2           13
#define MOTOR_B_IN1           25
#define MOTOR_B_IN2           14

#define MOTOR_LEDC_TIMER      LEDC_TIMER_0
#define MOTOR_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define MOTOR_LEDC_FREQ       17000
#define MOTOR_DUTY_MAX        80     /* 输出占空比上限(母线电压%), 反逻辑: 80%母线→20%PWM */

/* 旋向翻转: 左右电机镜像安装, 内部对该轮取反使"正duty=车体前进"语义统一
 * 现象修复: 左正右反 → 右电机翻转后两轮同向前进 */
#define MOTOR_A_INVERT        1     /* 电机A(右): 1=机械反装, 内部取反 */
#define MOTOR_B_INVERT        0     /* 电机B(左): 旋向正确, 不翻转 */

/* ==================== LED ==================== */
#define LED_GPIO              22
#define LED_LEDC_CH           LEDC_CHANNEL_4

/* ==================== 编码器(PCNT 脉冲计数) ==================== */
#define ENC_A_A              16    /* 电机A 编码器A */
#define ENC_A_B              17    /* 电机A 编码器B */
#define ENC_B_A              18    /* 电机B 编码器A */
#define ENC_B_B              19    /* 电机B 编码器B */
#define ENC_SPIKE_LIMIT      100   /* 相邻两次读数差超过此值视为尖峰 */
#define ENC_LPF_ALPHA        0.1f  /* 一阶低通滤波系数 (0~1, 越小越平滑) */

/* ==================== PID控制 ==================== */
#define MOTOR_LEFT_ENABLE     1     /* 1=使能左轮闭环输出, 0=禁用 */
#define MOTOR_RIGHT_ENABLE    1     /* 1=使能右轮闭环输出, 0=禁用 */

#define PID_LEFT_KP          -0.3f  /* 左轮增量PID Kp */
#define PID_LEFT_KI          -0.02f /* 左轮增量PID Ki */
#define PID_LEFT_KD          -0.0f  /* 左轮增量PID Kd */

#define PID_RIGHT_KP         -0.3f  /* 右轮增量PID Kp */
#define PID_RIGHT_KI         -0.02f /* 右轮增量PID Ki */
#define PID_RIGHT_KD         -0.0f  /* 右轮增量PID Kd */

#define CTRL_STARTUP_DELAY   1000   /* 启动等待周期数 (2ms × 1000 = 2000ms) */

/* ==================== 循迹速度控制 ==================== */
#define TRACK_BASE_SPEED          90 /* 共模目标转速(脉冲/2ms) */

/* 差模目标转速 — 质心(centroid)区间映射, 配合 controlLoop.c:trackSpeedFromState
 *
 * 正向 = 右转 (左轮加速/右轮减速), 负向 = 左转, 绝对值越大转弯越猛
 * 目标转速: leftTarget  = BASE_SPEED + DIFF/2
 *            rightTarget = BASE_SPEED - DIFF/2
 *
 *                            centroid  典型传感器位图     当前值
 * TRACK_DIFF_LEFT_SHARP   →  < -1.75   10000 (仅s0)      -190   最猛左转: 仅最外侧传感器/脱线搜寻
 * TRACK_DIFF_LEFT_FAR     → < -1.25    11000 (s0+s1)     -135   强左转:   两路外侧同时检测到
 * TRACK_DIFF_LEFT         → < -0.75    01000 (仅s1)       -88   中左转:   单路内侧检测到
 * TRACK_DIFF_LEFT_SLIGHT  → < -0.25    01100 (s1+s2)      -73   微左转:   内侧+中间检测到
 * TRACK_DIFF_CENTER       → [-0.25,+0.25]  00100 (仅s2)    0   直行
 * TRACK_DIFF_RIGHT_SLIGHT → > +0.25    00110 (s2+s3)      +73   微右转
 * TRACK_DIFF_RIGHT        → > +0.75    00010 (仅s3)       +88   中右转
 * TRACK_DIFF_RIGHT_FAR    → > +1.25    00011 (s3+s4)     +135   强右转
 * TRACK_DIFF_RIGHT_SHARP  → > +1.75    00001 (仅s4)      +190   最猛右转: 仅最外侧传感器/脱线搜寻 */

#define TRACK_DIFF_CENTER          0
#define TRACK_DIFF_LEFT_SLIGHT   -69
#define TRACK_DIFF_LEFT          -84
#define TRACK_DIFF_LEFT_FAR      -136
#define TRACK_DIFF_LEFT_SHARP    -190
#define TRACK_DIFF_RIGHT_SLIGHT   71
#define TRACK_DIFF_RIGHT          84
#define TRACK_DIFF_RIGHT_FAR      136
#define TRACK_DIFF_RIGHT_SHARP    190

/* diff 一阶低通滤波系数, 平滑差速输出跳变 */
#define TRACK_DIFF_LPF_ALPHA    0.25f

/* ==================== 数学宏 ==================== */
#define range(x, min, max)    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define absVal(x)             ((x) < 0 ? -(x) : (x))

/* ==================== Debug ==================== */
#define DEBUG_UART_NUM        UART_NUM_0
#define DEBUG_UART_TX         UART_PIN_NO_CHANGE
#define DEBUG_UART_BAUD       115200
#define DEBUG_SEND_INTERVAL   20    /* 每N次controlProcess发送一次 (2ms × 20 = 40ms) */

#endif /* WAYTRACK_COMMON_DEF_H_ */
