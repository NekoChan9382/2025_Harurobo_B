#include <mbed.h>

/*TODO
・NeoPixelのシリアル制御
・c_pushの削除(bit)
・新しいサーボの処理追加
*/

/// @Tips mm rad s
#include <PID_new.hpp>
#include <cmath>
#include <array>
#include <C620.hpp>
#include <Servo.hpp>


bool readline(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);

enum class c_state
{
    UP,
    DOWN,
    STOP
};

int main()
{
    printf("reset\n");
    DigitalOut ryugu(PA_6);
    DigitalOut emergency_to_arduio(PA_7);
    DigitalIn emergency_button(PB_6, PullUp);
    CAN can1(PA_11, PA_12, (int)1e6);
    int16_t pwm1[4] = {0, 0, 0, 0}; // pwm配列
    int16_t pwm2[4] = {0, 0, 0, 0}; // pwm配列
    // uint8_t servo1[8] = {0, 255, 170, 100, 0, 0, 0, 0};
    CANMessage msg1;
    CANMessage msg2;
    CANMessage msg_servo;
    constexpr int CAN_ID1 = 1; // 後で変える
    constexpr int CAN_ID2 = 2;
    constexpr int CAN_ID_SERVO = 149;

    constexpr int pid_max = 1;
    constexpr int dji_max_output = 8000;
    constexpr int motor_amount = 4;

    bool ag_fix = false;
    float fixed_angle = 0;

    int pos_mm[3] = {0}; // x, y, ang
    int motor_velocity[motor_amount] = {0};

    bool is_corn_indo_rolling = false;
    bool is_ball_indo_rolling = false;
    auto corn_conveyor_updown = c_state::STOP;
    bool is_ball_throwing = false;
    bool is_ball_conveyor_moving = false;
    auto corn_indo_updown = c_state::STOP;
    auto basket_updown = c_state::STOP;
    int corn_indo_updown_speed = 0;
    int corn_indo_roll_speed = 0;
    int ball_indo_roll_speed = 0;
    int corn_conveyor_speed = 0;
    int ball_conveyor_speed = 0;
    int ball_throw_speed = 0;
    int basket_updown_speed = 0;
    int servo_corn_deg = 0;
    int servo_seesaw_deg = 0;

    CAN can(PA_11, PA_12, 1000000);
    CANMessage msg_encoder;
    BufferedSerial pc(USBTX, USBRX, 115200);
    // BufferedSerial controller(PA_9, PA_10, 115200);
    dji::C620 c620(PB_12, PB_13);
    DigitalOut led(LED1);
    Servo servo_corn_contain(PC_6, 270, 2500us, 500us, 60);
    Servo servo_seesaw(PC_7, 180, 2500us, 500us, 60);

    PidGain pid_gain = {0.001, 0.00001, 0.0}; //0.0002, 0.0001, 0
    std::array<Pid, motor_amount> pid = {Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max})};

    PidGain pid_gain_angle = {0.001, 0.00001, 0.0};
    Pid pid_angle({pid_gain, -pid_max, pid_max});

    for (int i = 0; i < motor_amount; i++)
    {
        pid[i].reset();
    }
    pid_angle.reset();

    c620.set_max_output(dji_max_output);
    ryugu = 0;
    emergency_to_arduio = 0;
    printf("reset\n");

    while (true)
    {
        led=0;
        auto now = HighResClock::now();
        static auto pre = now;
        bool is_emergency_unlocked = emergency_button.read();
        if (is_emergency_unlocked == 0)
        {
            emergency_to_arduio = 1;
        }
        else
        {
            emergency_to_arduio = 0;
        }

        for (int i = 0; i < motor_amount; i++)
        {
            c620.read_data();
        }

        if (can.read(msg_encoder); msg_encoder.id == 10)
        {
            int16_t encoder_data[motor_amount] = {0};
            for (int i = 0; i < motor_amount; i++)
            {
                encoder_data[i] = msg_encoder.data[2 * i + 1] << 8 | msg_encoder.data[2 * i];
            }
            printf("enc_data,%d,%d,%d,%d\n", encoder_data[0], encoder_data[1], encoder_data[2], encoder_data[3]);
        }

        char data[15] = "";
        if (readline(pc, data) == 0)
        {
            if (strcmp(data, "vel") == 0)
            {
                for (int i = 0; i < motor_amount; i++)
                {
                    char data_vel[15] = "";
                    if (readline(pc, data_vel, true, false) == 0)
                    {
                        motor_velocity[i] = atoi(data_vel) * 19 * -1;
                    }
                }
            }

            if (strcmp(data, "sort_t") == 0)
            {
                servo_seesaw_deg = 112;
            }
            else if (strcmp(data, "sort_s") == 0)
            {
                servo_seesaw_deg = 87;
            }
            if (strcmp(data, "c_push") == 0)
            {
                if (servo_corn_deg == 0)
                {
                    servo_corn_deg = 180;
                }
                else
                {
                    servo_corn_deg = 0;
                }
            }
            if (strcmp(data, "b_conv") == 0)
            {
                is_ball_conveyor_moving = !is_ball_conveyor_moving;
            }
            if (strcmp(data, "c_conv_up") == 0)
            {
                corn_conveyor_updown = c_state::UP;
            }
            else if (strcmp(data, "c_conv_down") == 0)
            {
                corn_conveyor_updown = c_state::DOWN;
            }
            else if (strcmp(data, "c_conv_stop") == 0)
            {
                corn_conveyor_updown = c_state::STOP;
            }

            if (strcmp(data, "b_launch") == 0)
            {
                led=1;
                is_ball_throwing = true;
            }
            if (strcmp(data, "b_launch_s") == 0)
            {
                led=0;
                is_ball_throwing = false;
            }
            if (strcmp(data, "k_up") == 0)
            {
                basket_updown = c_state::UP;
            }
            else if (strcmp(data, "k_down") == 0)
            {
                basket_updown = c_state::DOWN;
            }
            else if (strcmp(data, "k_stop") == 0)
            {
                basket_updown = c_state::STOP;
            }

            if (strcmp(data, "c_indo") == 0)
            {
                //    printf("Pushed Circle\n");
                is_corn_indo_rolling = !is_corn_indo_rolling;
            }
            if (strcmp(data, "b_indo") == 0)
            {
                //    printf("Pushed Square\n");
                is_ball_indo_rolling = !is_ball_indo_rolling;
            }
            if (strcmp(data, "c_up") == 0)
            {
                corn_indo_updown = c_state::UP;
            }
            if (strcmp(data, "c_down") == 0)
            {
                corn_indo_updown = c_state::DOWN;
            }
            if (strcmp(data, "c_stop") == 0)
            {
                corn_indo_updown = c_state::STOP;
            }
            if (strcmp(data, "neo_push") == 0) //TODO: neo_pushに変更s
            {
                ryugu = !ryugu;
            }
            if (strcmp(data, "ag_fix_on") == 0)
            {
                ag_fix = true;
            }
            if (strcmp(data, "ag_fix_off") == 0)
            {
                ag_fix = false;
                pid_angle.reset();
            }
            if (strcmp(data, "ag_fix_val") == 0)
            {
                char data_ag[15] = "";
                
                if (readline(pc, data_ag, false, true) == 0)
                {
                    fixed_angle = atof(data_ag);

                    char data_cur_ag[15] = "";
                    if (readline(pc, data_cur_ag, false, true) == 0)
                    {
                        pos_mm[2] = atof(data_ag);
                    }
                }
            }


            if (corn_indo_updown == c_state::UP)
            {
                corn_indo_updown_speed = 6000;
            }
            else if (corn_indo_updown == c_state::DOWN)
            {
                corn_indo_updown_speed = -6000;
            }
            else
            {
                corn_indo_updown_speed = 0;
            }

            if (is_corn_indo_rolling)
            {
                corn_indo_roll_speed = -21000;
            }
            else
            {
                corn_indo_roll_speed = 0;
            }

            if (is_ball_indo_rolling)
            {
                ball_indo_roll_speed = -8000;
            }
            else
            {
                ball_indo_roll_speed = 0;
            }

            if (corn_conveyor_updown == c_state::UP)
            {
                corn_conveyor_speed = 13000;
            }
            else if (corn_conveyor_updown == c_state::DOWN)
            {
                corn_conveyor_speed = -13000;
            }
            else
            {
                corn_conveyor_speed = 0;
            }
            if (is_ball_conveyor_moving)
            {
                ball_conveyor_speed = -16000;
            }
            else
            {
                ball_conveyor_speed = 0;
            }
            if (is_ball_throwing)
            {
                ball_throw_speed = 13000;
            }
            else
            {
                ball_throw_speed = 0;
            }
            if (basket_updown == c_state::UP)
            {
                basket_updown_speed = 15000;
            }
            else if (basket_updown == c_state::DOWN)
            {
                basket_updown_speed = -15000;
            }
            else
            {
                basket_updown_speed = 0;
            }
            if (strcmp(data, "ping") == 0)
            {
                if (is_emergency_unlocked)
                {
                    printf("pong,eb_unlock\n");
                }
                else
                {
                    printf("pong,eb_lock\n");
                }
            }

            pwm1[0] = ball_conveyor_speed;
            pwm1[1] = ball_indo_roll_speed;
            pwm1[2] = ball_throw_speed;
            pwm1[3] = corn_indo_roll_speed;
            pwm2[0] = corn_indo_updown_speed;
            pwm2[1] = basket_updown_speed;
            pwm2[2] = 0;
            pwm2[3] = corn_conveyor_speed;
        
        }
        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("encoder_data: %d, %d, %d, %d\n", encoder_data[0], encoder_data[1], encoder_data[2], encoder_data[3]);

            int16_t motor_output[motor_amount] = {0};
            int motor_dpsa[motor_amount] = {0};
            int motor_goal[motor_amount] = {0};
            float motor_percent_modify = 0;

            if (ag_fix)
            {
                motor_percent_modify = pid_angle.calc(fixed_angle, pos_mm[2], elapsed);
            }

            for (int i = 0; i < motor_amount; i++)
            {
                float rmp_to_rad = 2 * M_PI / 60;
                float motor_dps = c620.get_rpm(i + 1) * rmp_to_rad;
                int goal_ang_vel = motor_velocity[i];
                const float percent = pid[i].calc(goal_ang_vel, motor_dps, elapsed);
                // printf("dps: %f, goal: %d, out: %d\n", motor_dps, goal_ang_vel, out);

                c620.set_output_percent(percent + motor_percent_modify, i + 1);
                motor_dpsa[i] = motor_dps * 180 / M_PI;
                motor_goal[i] = goal_ang_vel;
            }

            for (int i = 0; i < motor_amount; i++)
            {
                motor_output[i] = c620.get_current(i + 1);
            }
            printf("m_ou,%d,%d,%d,%d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            printf("m_vel,%d,%d,%d,%d\n", motor_dpsa[0], motor_dpsa[1], motor_dpsa[2], motor_dpsa[3]);
            printf("m_goal,%d,%d,%d,%d\n", motor_goal[0], motor_goal[1], motor_goal[2], motor_goal[3]);
            c620.write();
            
            CANMessage msg1(CAN_ID1, (const uint8_t *)&pwm1, 8);
            CANMessage msg2(CAN_ID2, (const uint8_t *)&pwm2, 8);
            CANMessage msg_servo(CAN_ID_SERVO, (const uint8_t *)&servo1, 8);
            can1.write(msg1);
            can1.write(msg2);
            can1.write(msg_servo);
            servo_corn_contain.move(servo_corn_deg);
            servo_seesaw.move(servo_seesaw_deg);

            pre = now;
        }
    }
}
bool readline(BufferedSerial &serial, char *buffer, const bool is_integar, const bool is_float)
{
    int i = 0;       // 繰り返し変数
    char buff = '0'; // シリアル受信

    if (not serial.readable())
    {
        return 1;
    }

    while ((buff != '\n') and i < 15)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信
        // printf("%c", buff);

        if (buff != '\n' && buff != '\r')
        {
            buffer[i] = buff; // 受信データ保存

            if (is_integar)
            {

                if (((buff < '0' || buff > '9') && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
            if (is_float)
            {

                if (((buff < '0' || buff > '9') && buff != '.' && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
        }
        i++;
    }
    // printf("\n");
    return 0;
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}

