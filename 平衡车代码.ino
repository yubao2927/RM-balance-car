/*
 * =====================================================================
 * 两轮自平衡小车固件 —— FreeRTOS + 串级PID / LQR 双算法版
 * =====================================================================
 *
 * 与原固件的兼容性：
 *   - 蓝牙/手机协议完全不变：
 *       A 前进   B 后退   C 左转   D 右转
 *       E / F 停止   G 特殊右转(原地)   H 特殊左转(原地)
 *       I 串口输出当前角度 angle
 *   - 上电默认运行串级PID（角度环PD 5ms + 速度环PI 40ms + 转向PD 20ms），
 *     参数与行为与原固件一致，手机操作无需任何改动。
 *
 * 新增功能（不影响原协议）：
 *   - 发送 'L' 切换到 LQR 状态反馈控制，发送 'P' 切回串级PID。
 *   - LQR 增益由 "工具/lqr_design.py" 离线解里卡提方程得到，
 *     修改物理参数后请重跑脚本并更新 LQR_K0..LQR_K4。
 *
 * FreeRTOS 结构（UNO 的 FreeRTOS 系统节拍为 15ms 看门狗节拍）：
 *   - 5ms 硬件定时中断(MsTimer2/Timer2)：姿态解算 + 控制律 + PWM，保证实时性；
 *   - FreeRTOS 任务 "Comm"：蓝牙串口解析(15ms) + 超声波避障(30ms)，
 *     与中断通过临界区共享指令变量。
 *
 * 依赖库（均已放入 库文件/）：FreeRTOS、MsTimer2、PinChangeInt、MPU6050、HCSR04
 * =====================================================================
 */

#include <Arduino_FreeRTOS.h>      // FreeRTOS 内核（必须第一个包含）
#include <MsTimer2.h>              // 5ms 控制定时器
#include <PinChangeInt.h>          // 任意引脚外部中断（编码器）
#include <MPU6050.h>               // MPU6050 库
#include <Wire.h>                  // I2C 通信
#include <HCSR04.h>                // 超声波库

MPU6050 mpu6050;
int16_t ax, ay, az, gx, gy, gz;

// ---------------- TB6612 引脚定义 ----------------
const int right_R1 = 12;
const int right_R2 = 13;
const int PWM_R   = 10;

const int left_L1  = 6;
const int left_L2  = 7;
const int PWM_L    = 9;

const int STBY = 8;

// ---------------- 姿态参数 ----------------
float angle_X;
float angle_Y;
float angle0 = 0;              // 机械平衡角，理想为 0 度
float Gyro_x, Gyro_y, Gyro_z;

// ---------------- 卡尔曼滤波器参数 ----------------
float Q_angle = 0.001;
float Q_gyro  = 0.003;
float R_angle = 0.5;
char  C_0 = 1;
float dt = 0.005;              // 与 5ms 定时中断一致
float K1 = 0.05;
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;

float accelz = 0;
float angle;
float angleY_one;
float angle_speed;
unsigned int CLB_u8trig;

float Pdot[4] = { 0, 0, 0, 0 };
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float PCt_0, PCt_1, E;

// ---------------- 串级PID参数（与原固件一致） ----------------
double kp = 38, ki = 0.0, kd = 0.58;              // 角度环 PD
double kp_speed = 3.8, ki_speed = 0.11, kd_speed = 0.0;   // 速度环 PI
double kp_turn = 15, ki_turn = 0, kd_turn = 0.15;         // 转向环 PD
double setp0 = 0;                                  // 角度平衡点
int PD_pwm;                                        // 角度环输出
float pwm1 = 0, pwm2 = 0;

// ---------------- LQR 控制参数 ----------------
// 由 工具/lqr_design.py 离线计算，状态 [倾角, 角速度, 位置, 速度, 位置积分]
// ctrl_sum = K0*angle + K1*angle_speed + K2*positions + K3*speeds_filter + K4*pos_int
// 符号约定与串级PID一致：倾角/角速度增益为正，位置/速度/积分增益为负
// （本车前进时位置积分为负积累，与速度环 -ki_speed*positions 同号）
const float LQR_K0 = 2.357593f;   // 每度
const float LQR_K1 = 0.206005f;   // 每度/秒
const float LQR_K2 = -0.002812f;  // 每编码脉冲
const float LQR_K3 = -0.128200f;  // 每(脉冲/40ms)
const float LQR_K4 = -0.000758f;  // 每(脉冲·秒)
float pos_int = 0.0f;             // 位置积分（仅 LQR 模式使用）
float ctrl_sum = 0.0f;            // 当前控制量（PID 或 LQR 的输出）

#define MODE_CASCADE_PID 0         // 串级PID（上电默认）
#define MODE_LQR         1         // LQR
volatile uint8_t control_mode = MODE_CASCADE_PID;

// ---------------- 中断速度计数 ----------------
#define PinA_left  2
#define PinA_right 4
volatile long count_right = 0;
volatile long count_left  = 0;
int speedcc = 0;

// ---------------- 脉冲计数 ----------------
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright, pulseleft;

// ---------------- 速度环变量 ----------------
float speeds_filterold = 0;
float positions = 0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;
float fultrasonic = 0;     // 超声波控制量
int Ultrasonicflag = 0;
float Distance = 0;

// ---------------- 转向 PD 变量 ----------------
int turnmax, turnmin, turnout;
float Turn_pwm = 0;
int zz = 0;
int turncc = 0;

// ---------------- 蓝牙控制指令（手机协议，保持不变） ----------------
int carfront = 0;      // 前进
int carback  = 0;      // 后退
int carleft  = 0;      // 左转
int carright = 0;      // 右转
char val;

HCSR04 hc(A2, A3);             // trig, echo

// =====================================================================
// FreeRTOS 通信任务：蓝牙串口 + 超声波避障
// =====================================================================
void TaskComm(void *pvParameters)
{
  (void) pvParameters;
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t tick = 0;

  for (;;)
  {
    // ---- 蓝牙串口解析：协议与原固件完全一致 ----
    while (Serial.available())
    {
      val = Serial.read();
      switch (val)
      {
        case 'A':
          taskENTER_CRITICAL();
          carfront = -150; Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 前进
        case 'B':
          taskENTER_CRITICAL();
          carback = 150; Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 后退
        case 'C':
          taskENTER_CRITICAL();
          carright = 1; Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 左转
        case 'D':
          taskENTER_CRITICAL();
          carleft = 1; Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 右转
        case 'E':
        case 'F':
          taskENTER_CRITICAL();
          carfront = 0; carback = 0; carleft = 0; carright = 0;
          Ultrasonicflag = 0;
          taskEXIT_CRITICAL();
          break;                                // 停止
        case 'H':
          taskENTER_CRITICAL();
          carfront = 0; carback = 0; carleft = 1; carright = 1;
          Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 特殊左转（原地）
        case 'G':
          taskENTER_CRITICAL();
          carfront = 0; carback = 0; carleft = -1; carright = -1;
          Ultrasonicflag = 1;
          taskEXIT_CRITICAL();
          break;                                // 特殊右转（原地）
        case 'I':
          Serial.print(angle);
          Serial.println();
          break;                                // 查询角度

        // ---- 新增：控制算法切换（手机APP不会发送这两个字符，原协议不受影响） ----
        case 'P':
        case 'p':
          control_mode = MODE_CASCADE_PID;      // 切回串级PID
          break;
        case 'L':
        case 'l':
          taskENTER_CRITICAL();
          pos_int = 0.0f;                        // 切换时清零位置积分
          taskEXIT_CRITICAL();
          control_mode = MODE_LQR;               // 切换LQR
          break;
        default:
          break;
      }
    }

    // ---- 超声波避障：每 30ms 一次 ----
    tick++;
    if ((tick % 2) == 0)
    {
      UltraSonicProcessing();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(15));
  }
}

// =====================================================================
// 初始化
// =====================================================================
void setup()
{
  // 电机控制引脚
  pinMode(right_R1, OUTPUT);
  pinMode(right_R2, OUTPUT);
  pinMode(left_L1, OUTPUT);
  pinMode(left_L2, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(right_R1, 1);
  digitalWrite(right_R2, 0);
  digitalWrite(left_L1, 0);
  digitalWrite(left_L2, 1);
  digitalWrite(STBY, 1);                     // 芯片使能

  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);

  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);

  Ultrasonicflag = 0;
  fultrasonic = 0;
  CLB_u8trig = 0;

  Wire.begin();
  Serial.begin(9600);
  delay(1500);                               // 上电静置，卡尔曼初始化
  mpu6050.initialize();
  delay(2);

  // 编码器外部中断（原固件在 loop 里反复 attach，这里只 attach 一次）
  attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);

  // 5ms 控制中断：姿态解算 + 控制律 + PWM（保证控制实时性）
  MsTimer2::set(5, TimingInterrupt);
  MsTimer2::start();

  // FreeRTOS 通信任务
  xTaskCreate(TaskComm, "Comm", 256, NULL, 2, NULL);

  vTaskStartScheduler();                     // 调度器接管，不会再返回

  for (;;) { }                               // 理论上不可达
}

void loop()
{
  // FreeRTOS 调度器启动后，所有工作都在任务和中断中完成
}

// =====================================================================
// 编码器计数（引脚变化中断）
// =====================================================================
void Code_left()
{
  count_left++;
}

void Code_right()
{
  count_right++;
}

// ---------------- 脉冲统计（5ms 一次） ----------------
void countpluse()
{
  lz = count_left;
  rz = count_right;

  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) && (pwm2 < 0))          // 后退：脉冲记负
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0))     // 前进
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0))     // 左转
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0))     // 右转
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  pulseright += rpluse;
  pulseleft  += lpluse;
}

// =====================================================================
// 5ms 控制中断：传感器 -> 滤波 -> 控制律 -> PWM
// =====================================================================
void TimingInterrupt()
{
  sei();                                               // 允许中断嵌套
  countpluse();
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

  cc++;
  if (cc >= 8)                                         // 40ms：速度环/位置状态更新
  {
    carspeedpiout();
    cc = 0;
  }

  // ---- 控制律选择：串级PID 或 LQR ----
  if (control_mode == MODE_LQR)
  {
    pos_int += positions * dt;                         // 位置积分（与 lqr_design.py 一致）
    if (pos_int > 60000.0f) pos_int = 60000.0f;        // 抗积分饱和
    if (pos_int < -60000.0f) pos_int = -60000.0f;

    ctrl_sum = LQR_K0 * (angle + angle0)
             + LQR_K1 * angle_speed
             + LQR_K2 * positions
             + LQR_K3 * speeds_filter
             + LQR_K4 * pos_int;
  }
  else
  {
    AnglePD();                                         // 角度环 PD（5ms）
    ctrl_sum = (float) PD_pwm + PI_pwm;                // 角度环 + 速度环
  }

  turncc++;
  if (turncc > 4)                                      // 20ms：转向环
  {
    carturnspin();
    turncc = 0;
  }

  anglePWM();
}

// =====================================================================
// 姿态解算
// =====================================================================
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float dt, float Q_angle, float Q_gyro, float R_angle,
                     float C_0, float K1)
{
  float Angle = -atan2(ay, az) * (180 / PI);           // 倾角
  Gyro_x = -gx / 131.0;                                // x 轴角速度
  Kalman_Filter(Angle, Gyro_x);                        // 卡尔曼滤波

  Gyro_z = -gz / 131.0;                                // z 轴角速度（转向）

  float angleAx = -atan2(ax, az) * (180 / PI);         // y 轴倾角
  Gyro_y = -gy / 131.0;
  Yiorderfilter(angleAx, Gyro_y);                      // 一阶互补滤波
}

// ---------------- 卡尔曼滤波器 ----------------
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;                     // 先验估计
  angle_err = angle_m - angle;

  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;

  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];

  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;

  q_bias += K_1 * angle_err;
  angle_speed = gyro_m - q_bias;                       // 最优角速度
  angle += K_0 * angle_err;                            // 最优角度
}

// ---------------- 一阶滤波 ----------------
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

// =====================================================================
// 串级PID：内环角度 PD
// =====================================================================
void AnglePD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed;
}

// =====================================================================
// 串级PID：外环速度/位置 PI（40ms）
// 同时为 LQR 提供 positions、speeds_filter 状态
// =====================================================================
void carspeedpiout()
{
  float speeds = (pulseleft + pulseright) * 1.0;
  pulseright = pulseleft = 0;

  speeds_filterold *= 0.7;                             // 一阶互补滤波
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;

  positions += speeds_filter;
  positions += carfront;                               // 前进指令融合
  positions += carback;                                // 后退指令融合
  positions += fultrasonic;                            // 超声波避障融合
  positions = constrain(positions, -3550, 3550);       // 抗积分饱和

  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);
}

// =====================================================================
// 转向 PD（20ms）
// =====================================================================
void carturnspin()
{
  int flag = 0;
  float turnspeed = 0;
  float rotationratio = 0;

  if (carleft == 1 || carright == 1 || carleft == -1 || carright == -1)
  {
    if (flag == 0)
    {
      turnspeed = (pulseright + pulseleft);
      flag = 1;
    }
    if (turnspeed < 0)
    {
      turnspeed = -turnspeed;
    }
    if (carleft == 1 || carright == 1)
    {
      turnmax = 2;
      turnmin = -2;
    }
    rotationratio = 3 / turnspeed;
    if (rotationratio < 0.5) rotationratio = 0.5;
    if (rotationratio > 5) rotationratio = 5;
  }
  else
  {
    rotationratio = 0.5;
    flag = 0;
    turnspeed = 0;
  }

  if (carleft == 1)
  {
    turnout += rotationratio;
  }
  else if (carright == 1)
  {
    turnout -= rotationratio;
  }
  else if (carleft == -1)
  {
    turnout -= rotationratio;
  }
  else turnout = 0;

  if (turnout > turnmax) turnout = turnmax;
  if (turnout < turnmin) turnout = turnmin;

  Turn_pwm = -turnout * kp_turn - Gyro_z * kd_turn;
}

// =====================================================================
// PWM 合成与输出
// =====================================================================
void anglePWM()
{
  pwm2 = -ctrl_sum + Turn_pwm;                         // 左电机
  pwm1 = -ctrl_sum - Turn_pwm;                         // 右电机

  if (pwm1 > 255) pwm1 = 255;
  if (pwm1 < -255) pwm1 = -255;
  if (pwm2 > 255) pwm2 = 255;
  if (pwm2 < -255) pwm2 = -255;

  if (angle > 45 || angle < -45)                       // 倾角过大保护
  {
    pwm1 = pwm2 = 0;
  }

  if (pwm2 >= 0)
  {
    digitalWrite(left_L1, LOW);
    digitalWrite(left_L2, HIGH);
    analogWrite(PWM_L, pwm2);
  }
  else
  {
    digitalWrite(left_L1, HIGH);
    digitalWrite(left_L2, LOW);
    analogWrite(PWM_L, -pwm2);
  }

  if (pwm1 >= 0)
  {
    digitalWrite(right_R1, LOW);
    digitalWrite(right_R2, HIGH);
    analogWrite(PWM_R, pwm1);
  }
  else
  {
    digitalWrite(right_R1, HIGH);
    digitalWrite(right_R2, LOW);
    analogWrite(PWM_R, -pwm1);
  }
}

// =====================================================================
// 超声波避障：2cm < 距离 < 6cm 时向位置积分注入 +250 后退
// =====================================================================
void UltraSonicProcessing(void)
{
  if (Ultrasonicflag == 0)
  {
    Distance = hc.dist();
    if (Distance > 2.0 && Distance < 6.00)
    {
      fultrasonic = 250;
    }
    else
    {
      fultrasonic = 0;
    }
  }
}
