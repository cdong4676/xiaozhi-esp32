#include "otto_movements.h"

#include <algorithm>

#include "oscillator.h"

static const char* TAG = "OttoMovements";

#define HAND_HOME_POSITION 90

Otto::Otto() {
    is_otto_resting_ = false;
    has_hands_ = false;
    is_wheel_mode_ = false;
    // 初始化所有舵机管脚为-1（未连接）
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Otto::~Otto() {
    DetachServos();
}

unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void Otto::Init(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand,
                int right_hand) {
    servo_pins_[LEFT_LEG] = left_leg;
    servo_pins_[RIGHT_LEG] = right_leg;
    servo_pins_[LEFT_FOOT] = left_foot;
    servo_pins_[RIGHT_FOOT] = right_foot;
    servo_pins_[LEFT_HAND] = left_hand;
    servo_pins_[RIGHT_HAND] = right_hand;

    // 检查是否有手部舵机
    has_hands_ = (left_hand != -1 && right_hand != -1);

    AttachServos();
    is_otto_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::AttachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            // 左右foot舵机设置为连续旋转模式
           
            servo_[i].Attach(servo_pins_[i], false);
            if (i == LEFT_FOOT || i==RIGHT_FOOT)
            {
               servo_[i].SetPosition(90);
            }
        }
    }
}

void Otto::DetachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Detach();
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::SetTrims(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand,
                    int right_hand) {
    servo_trim_[LEFT_LEG] = left_leg;
    servo_trim_[RIGHT_LEG] = right_leg;
    servo_trim_[LEFT_FOOT] = left_foot;
    servo_trim_[RIGHT_FOOT] = right_foot;

    if (has_hands_) {
        servo_trim_[LEFT_HAND] = left_hand;
        servo_trim_[RIGHT_HAND] = right_hand;
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetTrim(servo_trim_[i]);
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::MoveServos(int time, int servo_target[]) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if (time > 10) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 ) {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
            }
        }

        for (int iteration = 1; millis() < final_time_; iteration++) {
            partial_time_ = millis() + 10;
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1 ) {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 ) {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // final adjustment to the target.
    bool f = true;
    int adjustment_count = 0;
    while (f && adjustment_count < 10) {
        f = false;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition()) {
                f = true;
                break;
            }
        }
        if (f) {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1 ) {
                    servo_[i].SetPosition(servo_target[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            adjustment_count++;
        }
    };
}

void Otto::MoveSingle(int position, int servo_number) {
    if (position > 180)
        position = 90;
    if (position < 0)
        position = 90;

    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1) {
        servo_[servo_number].SetPosition(position);
    }
}

void Otto::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                           double phase_diff[SERVO_COUNT], float cycle = 1) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Otto::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                   double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    //-- Execute the final not complete cycle
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Otto at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::Home(bool hands_down) {
    if (is_otto_resting_ == false) {  // Go to rest position only if necessary
        // 为所有舵机准备初始位置值
        int homes[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (i == LEFT_HAND || i == RIGHT_HAND) {
                if (hands_down) {
                    // 如果需要复位手部，设置为默认值
                    if (i == LEFT_HAND) {
                        homes[i] = HAND_HOME_POSITION;
                    } else {                                  // RIGHT_HAND
                        homes[i] = HAND_HOME_POSITION;  // 右手镜像位置
                    }
                } else {
                    // 如果不需要复位手部，保持当前位置
                    homes[i] = servo_[i].GetPosition();
                }
            }else {
                // 腿部和脚部舵机始终复位
                if (is_wheel_mode_)
                {
                    if (i == LEFT_LEG)
                    {
                        homes[i] = 180;
                    }else if (i== RIGHT_LEG)
                    {
                        homes[i] = 0;
                    }else{
                        homes[i] = 90;
                    }
                }else{
                    homes[i] = 90;
                }
                
            }
        }

        MoveServos(500, homes);
        is_otto_resting_ = true;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}
bool Otto::GetRestState() {
    return is_otto_resting_;
}

void Otto::SetRestState(bool state) {
    is_otto_resting_ = state;
}
//------------------------------------------------------------------
//-- Otto gait: Walking  (forward or backward)
//--  Paro
//rection: FORWARD / BACKWARD
//--    * amount: 手部摆动幅度, 0表示不摆动
//---------------------------------------------------------
void Otto::Walk(float steps, int period, int dir, int spin_foot) {
    // 检查当前模式
    if (is_wheel_mode_) {
        if (GetRestState()) {
            SetRestState(false);
        }
        // 轮子模式：使用foot舵机进行滚动
        int left_speed, right_speed;
        
        // 根据方向设置轮子速度
        if (dir == FORWARD) {
            left_speed = 50;   // 前进：左轮正转
            right_speed = -50; // 前进：右轮反转
        } else { // BACKWARD
            left_speed = -50;  // 后退：左轮反转
            right_speed = 50;  // 后退：右轮正转
        }
        
        // 设置foot旋转速度
        SetFootSpeed(left_speed, right_speed);
        
        // 根据steps计算运行时间
        int run_time = (int)(steps * period);
        vTaskDelay(pdMS_TO_TICKS(run_time));
        
        // 停止旋转
        StopFoot();
        
        ESP_LOGI(TAG, "Wheel mode walk: dir=%d, steps=%.1f, time=%dms", dir, steps, run_time);
    } else {
        // 普通脚模式：使用基于时间间隔的步态序列
        /* if (spin_foot == 1)
        {
            // 调用普通脚模式下的旋转轮子来实现移动的函数
            FootModeSpinWalk(steps, period, dir);
        }else{
            WalkModeSequence(steps, period, dir, 0);  
        } */
        WalkModeSequence(steps, period, dir, 0);  
        
        
        ESP_LOGI(TAG, "Foot mode walk: dir=%d, steps=%.1f, period=%d", dir, steps, period);
    }
}

//---------------------------------------------------------
//-- Otto gait: Turning (left or right)
//--  Parameters:
//--   * Steps: Number of steps
//--   * T: Period
//--   * Dir: Direction: LEFT / RIGHT
//--   * amount: 手部摆动幅度, 0表示不摆动
//---------------------------------------------------------
void Otto::Turn(float steps, int period, int dir, int amount) {
    // 检查当前模式
    if (is_wheel_mode_) {
        if (GetRestState()) {
            SetRestState(false);
        }
        // 轮子模式：使用差速转向
        int left_speed, right_speed;
        
        // 根据转向方向设置轮子速度
        if (dir == LEFT) {
            left_speed = -30;  // 左转：左轮反转
            right_speed = -30; // 左转：右轮反转（同向转向）
        } else { // RIGHT
            left_speed = 30;   // 右转：左轮正转
            right_speed = 30;  // 右转：右轮正转（同向转向）
        }
        
        // 设置foot旋转速度
        SetFootSpeed(left_speed, right_speed);
        
        // 根据steps计算运行时间
        int run_time = (int)(steps * (period/2));
        vTaskDelay(pdMS_TO_TICKS(run_time));
        
        // 停止旋转
        StopFoot();
        
        ESP_LOGI(TAG, "Wheel mode turn: dir=%d, steps=%.1f, time=%dms", dir, steps, run_time);
    } else {
        
        WalkModeSequence(steps, period, dir, amount);
        ESP_LOGI(TAG, "Foot mode turn: dir=%d, steps=%.1f, period=%d", dir, steps, period);
    }
}

void Otto::EnableServoLimit(int diff_limit) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetLimiter(diff_limit);
        }
    }
}

void Otto::DisableServoLimit() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].DisableLimiter();
        }
    }
}

//---------------------------------------------------------
//-- 轮子模式: 设置左右foot旋转速度
//--  Parameters:
//--    left_speed: 左foot速度 (-100到100)
//--    right_speed: 右foot速度 (-100到100)
//---------------------------------------------------------
void Otto::SetFootSpeed(int left_speed, int right_speed) {
    if (servo_pins_[LEFT_FOOT] != -1 ) {
        servo_[LEFT_FOOT].SetSpeed(left_speed);
    }
    if (servo_pins_[RIGHT_FOOT] != -1) {
        servo_[RIGHT_FOOT].SetSpeed(right_speed);
    }
}

//---------------------------------------------------------
//-- 轮子模式: 停止foot旋转
//---------------------------------------------------------
void Otto::StopFoot() {
    if (servo_pins_[LEFT_FOOT] != -1 ) {
        servo_[LEFT_FOOT].StopRotation();
    }
    if (servo_pins_[RIGHT_FOOT] != -1 ) {
        servo_[RIGHT_FOOT].StopRotation();
    }
}

//---------------------------------------------------------
//-- 轮子模式: 进入轮子模式
//-- 将leg调到合适位置，并重新配置foot舵机为连续旋转模式
//---------------------------------------------------------
void Otto::WheelMode() {
    if (is_wheel_mode_) {
        return;  // 已经在轮子模式
    }

    if(has_hands_) {
        servo_[LEFT_HAND].SetPosition(50);
        servo_[RIGHT_HAND].SetPosition(130);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 明确发送停止信号
    if (servo_pins_[LEFT_FOOT] != -1) {
        servo_[LEFT_FOOT].SetPosition(90);  // 90度为停止位置
    }
    if (servo_pins_[RIGHT_FOOT] != -1) {
        servo_[RIGHT_FOOT].SetPosition(90);  // 90度为停止位置
    }
    
    // 设置leg到合适位置以支持轮子模式
    if (servo_pins_[LEFT_LEG] != -1) {
        servo_[LEFT_LEG].SetPosition(180);  // 左leg调到180度
    }
    if (servo_pins_[RIGHT_LEG] != -1) {
        servo_[RIGHT_LEG].SetPosition(0);   // 右leg调到0度
    }
    

    vTaskDelay(pdMS_TO_TICKS(200));  // 等待舵机移动到位
    if(has_hands_) {
        servo_[LEFT_HAND].SetPosition(90);
        servo_[RIGHT_HAND].SetPosition(90);
    }
    
    is_wheel_mode_ = true;
    is_otto_resting_ = false;
    
    ESP_LOGI(TAG, "Entered wheel mode - legs and feet adjusted");
}

//---------------------------------------------------------
//-- 恢复正常foot模式
//-- 将leg调回home位置，并重新配置foot舵机为位置控制模式
//---------------------------------------------------------
void Otto::FootMode() {
    if (!is_wheel_mode_) {
        return;  // 不在轮子模式，无需切换
    }

    // 停止foot旋转
    StopFoot();
    
    servo_[LEFT_FOOT].SetPosition(90);  // 设置到中性位置
    servo_[RIGHT_FOOT].SetPosition(90);  // 设置到中性位置

    // 将leg调回home位置（90度）
    if (servo_pins_[LEFT_LEG] != -1) {
        servo_[LEFT_LEG].SetPosition(90);  // 左leg回到home位置
    }
    if (servo_pins_[RIGHT_LEG] != -1) {
        servo_[RIGHT_LEG].SetPosition(90);  // 右leg回到home位置
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));  // 等待舵机移动到位
    
    is_wheel_mode_ = false;  // 退出轮子模式
    
    ESP_LOGI(TAG, "Switched to normal foot mode - legs and feet adjusted");
}


bool Otto::IsWheelMode() const {
    return is_wheel_mode_;
}

//---------------------------------------------------------
//-- 基于时间间隔的步态序列 (参考Arduino Walkmode)
//--  Parameters:
//--    * steps:  Number of steps
//--    * period : Period for each step cycle
//--    * dir: Direction: FORWARD / BACKWARD
//--    * amount: 手部摆动幅度, 0表示不摆动
//---------------------------------------------------------
void Otto::WalkModeSequence(float steps, int period, int dir, int amount) {
    if (GetRestState()) {
        SetRestState(false);
    }
    
    // 执行指定步数的步态循环
    for (int step = 0; step < static_cast<int>(steps); step++) {
        int cycle_start = 0;
        
        // 步态序列循环
        while ( cycle_start< 8) {
            
            ExecuteGaitPhase(cycle_start, dir);
            vTaskDelay(pdMS_TO_TICKS(GAIT_LOOP_DELAY_MS ));  // 增加延迟时间，让步态更慢
            cycle_start++;
        }
    }
    
    // 步态完成后回到home位置
    Home(false);  // 不重置手部位置
}

//---------------------------------------------------------
//-- 执行特定步态阶段
//---------------------------------------------------------
void Otto::ExecuteGaitPhase(int phase, int dir) {
    int foodSpeed =120; 
    switch (phase) {
        case 0: // 右腿抬起，左腿支撑
            
            SetLegPosition(RIGHT_LEG, GAIT_INITIAL_ANGLE + GAIT_LEG_LIFT_ANGLE);
            SetLegPosition(LEFT_LEG, GAIT_INITIAL_ANGLE + GAIT_LEG_SUPPORT_ANGLE);
            break;
            
        case 1: // 右脚推进
            SetLegPosition(RIGHT_LEG, GAIT_INITIAL_ANGLE + GAIT_LEG_PUSH_ANGLE);
            
            if (dir== FORWARD)
            {
                foodSpeed=120;
            }else if (dir == BACKWARD)
            {
                foodSpeed = 60;
            }else if(dir== LEFT){
                foodSpeed = 100;
            }else{
                foodSpeed = 80;
            }
            SetFootPosition(LEFT_FOOT, foodSpeed);
            break;
            
        case 2: // 右脚停止推进
            SetFootPosition(LEFT_FOOT, GAIT_FOOT_NEUTRAL_ANGLE);
            break;
            
        case 3: // 双腿回到中性位置
            SetLegPosition(LEFT_LEG, GAIT_INITIAL_ANGLE);
            SetLegPosition(RIGHT_LEG, GAIT_INITIAL_ANGLE);
            break;
            
        case 4: // 左腿抬起，右腿支撑
            SetLegPosition(LEFT_LEG, GAIT_INITIAL_ANGLE - GAIT_LEG_LIFT_ANGLE);
            SetLegPosition(RIGHT_LEG, GAIT_INITIAL_ANGLE - GAIT_LEG_SUPPORT_ANGLE);
            break;
            
        case 5: // 左脚推进
            SetLegPosition(LEFT_LEG, GAIT_INITIAL_ANGLE -GAIT_LEG_PUSH_ANGLE);
             if (dir== FORWARD)
            {
                foodSpeed=60;
            }else if (dir == BACKWARD)
            {
                foodSpeed = 120;
            }else if(dir== LEFT){
                foodSpeed = 100;
            }else{
                foodSpeed = 80;
            }
            SetFootPosition(RIGHT_FOOT, foodSpeed);
            break;

            
        case 6: // 左脚停止推进
            SetFootPosition(RIGHT_FOOT, GAIT_FOOT_NEUTRAL_ANGLE);
            break;
            
        case 7: // 双腿回到中性位置
            SetLegPosition(LEFT_LEG, GAIT_INITIAL_ANGLE);
            SetLegPosition(RIGHT_LEG, GAIT_INITIAL_ANGLE);
            break;
            
        default:
            break;
    }
}

//---------------------------------------------------------
//-- 设置腿部位置的辅助函数
//---------------------------------------------------------
void Otto::SetLegPosition(int leg_index, int angle) {
    if (servo_pins_[leg_index] != -1) {
        servo_[leg_index].SetPosition(angle);
    }
}

//---------------------------------------------------------
//-- 设置脚部位置的辅助函数
//---------------------------------------------------------
void Otto::SetFootPosition(int foot_index, int angle) {
    if (servo_pins_[foot_index] != -1 ) {
        servo_[foot_index].SetPosition(angle);
    }
}

///////////////////////////////////////////////////////////////////
//-- HAND WAVING FUNCTIONS --------------------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- 攻击函数
//--  Parameters:
//--    * dir: 挥哪一只手 LEFT/RIGHT/BOTH
//--    * cycles: 挥动次数
//--    * period: 每次挥动的周期时间(ms)
//---------------------------------------------------------
void Otto::Attack(int dir, int cycles, int period) {
    if (!has_hands_) {
        ESP_LOGW(TAG, "Hand servos not available");
        return;
    }

    // 检查具体舵机可用性
    bool left_available = (servo_pins_[LEFT_HAND] != -1);
    bool right_available = (servo_pins_[RIGHT_HAND] != -1);
    
    if ((dir == LEFT && !left_available) || 
        (dir == RIGHT && !right_available) ||
        (dir == BOTH && (!left_available || !right_available))) {
        ESP_LOGW(TAG, "Required hand servo(s) not available for direction %d", dir);
        return;
    }

    if (GetRestState()) {
        SetRestState(false);
    }

    // 向上挥动
        if (dir == LEFT || dir == BOTH) {
            servo_[LEFT_HAND].SetPosition(30);
        }
        if (dir == RIGHT || dir == BOTH) {
            servo_[RIGHT_HAND].SetPosition(150);
        }
        vTaskDelay(pdMS_TO_TICKS(period / 2));
        
    
    // 回到初始位置
    if (dir == LEFT || dir == BOTH) {
        servo_[LEFT_HAND].SetPosition(90);
    }
    if (dir == RIGHT || dir == BOTH) {
        servo_[RIGHT_HAND].SetPosition(90);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
}

//---------------------------------------------------------
//-- 普通脚模式下的轮子旋转移动
//--  Parameters:
//--    * steps: 移动步数（影响运行时间）
//--    * period: 每步的时间周期(ms)
//--    * dir: 移动方向: FORWARD / BACKWARD / LEFT / RIGHT
//---------------------------------------------------------
void Otto::FootModeSpinWalk(float steps, int period, int dir) {
    if (is_wheel_mode_) {
        ESP_LOGW(TAG, "Cannot use FootModeSpinWalk in wheel mode");
        return;
    }
    
    if (GetRestState()) {
        SetRestState(false);
    }
    
    /* // 确保腿部在正常位置（90度）
    if (servo_pins_[LEFT_LEG] != -1) {
        servo_[LEFT_LEG].SetPosition(90);
    }
    if (servo_pins_[RIGHT_LEG] != -1) {
        servo_[RIGHT_LEG].SetPosition(90);
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));  // 等待腿部调整到位 */
    
    // 根据方向设置foot舵机的旋转速度
    int left_speed, right_speed;
    
    switch (dir) {
        case FORWARD:
            left_speed = 40;   // 前进：左foot正转
            right_speed = -40; // 前进：右foot反转
            break;
            
        case BACKWARD:
            left_speed = -40;  // 后退：左foot反转
            right_speed = 40;  // 后退：右foot正转
            break;
            
        case LEFT:
            left_speed = -30;  // 左转：左foot反转
            right_speed = -30; // 左转：右foot反转（同向转向）
            break;
            
        case RIGHT:
            left_speed = 30;   // 右转：左foot正转
            right_speed = 30;  // 右转：右foot正转（同向转向）
            break;
            
        default:
            ESP_LOGW(TAG, "Invalid direction for FootModeSpinWalk: %d", dir);
            return;
    }
    
    // 设置foot旋转速度
    SetFootSpeed(left_speed, right_speed);
    
    // 根据steps和period计算运行时间
    int run_time = (int)(steps * period);
    vTaskDelay(pdMS_TO_TICKS(run_time));
    
    // 停止旋转
    StopFoot();
    
    ESP_LOGI(TAG, "Foot mode spin walk: dir=%d, steps=%.1f, time=%dms", dir, steps, run_time);
}
