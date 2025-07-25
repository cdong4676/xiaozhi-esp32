#ifndef __OTTO_MOVEMENTS_H__
#define __OTTO_MOVEMENTS_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oscillator.h"

//-- Constants
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define BOTH 5

// -- Servo delta limit default. degree / sec
#define SERVO_LIMIT_DEFAULT 240

// -- Servo indexes for easy access
#define LEFT_LEG 0
#define RIGHT_LEG 1
#define LEFT_FOOT 2
#define RIGHT_FOOT 3
#define LEFT_HAND 4
#define RIGHT_HAND 5
#define SERVO_COUNT 6

// -- 步态常量定义
#define GAIT_INITIAL_ANGLE 90
#define GAIT_LEG_LIFT_ANGLE 60
#define GAIT_LEG_SUPPORT_ANGLE 30
#define GAIT_LEG_PUSH_ANGLE 50
#define GAIT_LEG_FINAL_ANGLE 60
#define GAIT_FOOT_FORWARD_ANGLE 120
#define GAIT_FOOT_BACKWARD_ANGLE 60
#define GAIT_FOOT_NEUTRAL_ANGLE 90
#define GAIT_PHASES_COUNT 8
#define GAIT_LOOP_DELAY_MS 300

class Otto {
public:
    Otto();
    ~Otto();

    //-- Otto initialization
    void Init(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand = -1,
              int right_hand = -1);
    //-- Attach & detach functions
    void AttachServos();
    void DetachServos();

    //-- Oscillator Trims
    void SetTrims(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand = 0,
                  int right_hand = 0);

    //-- Predetermined Motion Functions
    void MoveServos(int time, int servo_target[]);
    void MoveSingle(int position, int servo_number);
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                         double phase_diff[SERVO_COUNT], float cycle);

    //-- HOME = Otto at rest position
    void Home(bool hands_down = true);
    bool GetRestState();
    void SetRestState(bool state);

    //-- Predetermined Motion Functions

    void Walk(float steps = 4, int period = 1000, int dir = FORWARD, int spin_foot = 0);
    void Turn(float steps = 4, int period = 2000, int dir = LEFT, int amount = 0);
    
    // -- 基于时间间隔的步态序列
    void WalkModeSequence(float steps, int period, int dir, int amount);
    void ExecuteGaitPhase(int phase, int dir);
    void SetLegPosition(int leg_index, int angle);
    void SetFootPosition(int foot_index, int angle);
    
    // -- 普通脚模式下的轮子旋转移动
    void FootModeSpinWalk(float steps, int period, int dir);

   
    // -- 轮子模式动作 (360度连续旋转舵机)
    void SetFootSpeed(int left_speed, int right_speed);  // 设置左右foot旋转速度 (-100到100)
    void StopFoot();                                     // 停止foot旋转
    void WheelMode();                                    // 进入轮子模式：leg调到90度，foot开始旋转
    void FootMode();                                     // 恢复正常foot模式：将连续旋转舵机切换回位置控制模式

    // -- Servo limiter
    void EnableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void DisableServoLimit();

    // -- 状态查询
    bool IsWheelMode() const;  // 获取当前是否处于轮子模式

    // -- 手部动作功能
    void Attack(int dir = BOTH, int cycles = 1, int period = 500);  // 挥手动作：dir可选LEFT/RIGHT/BOTH

private:
    Oscillator servo_[SERVO_COUNT];

    int servo_pins_[SERVO_COUNT];
    int servo_trim_[SERVO_COUNT];

    unsigned long final_time_;
    unsigned long partial_time_;
    float increment_[SERVO_COUNT];

    bool is_otto_resting_;
    bool has_hands_;     // 是否有手部舵机
    bool is_wheel_mode_; // 是否处于轮子模式

    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                 double phase_diff[SERVO_COUNT], float steps);
};

#endif  // __OTTO_MOVEMENTS_H__