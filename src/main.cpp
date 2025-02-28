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
    DigitalOut ryugu(PA_6);
    CAN can1(PA_11, PA_12, (int)1e6);
    int16_t pwm1[4] = {0, 0, 0, 0}; // pwm配列
    int16_t pwm2[4] = {0, 0, 0, 0}; // pwm配列
    int8_t servo1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    CANMessage msg1;
    CANMessage msg2;
    CANMessage msg_servo;
    constexpr int CAN_ID1 = 2; // 後で変える
    constexpr int CAN_ID2 = 3;
    constexpr int CAN_ID_SERVO = 141;

    constexpr int pid_max = 1;
    constexpr int dji_max_output = 8000;
    constexpr int motor_amount = 4;

    int pos_mm[3] = {0}; // x, y, ang
    int motor_velocity[motor_amount] = {0};

    bool is_c_indoroll = false;
    bool is_b_indoroll = false;
    bool c_move_belt = false;
    bool is_b_throw = false;
    auto c_direction = c_state::STOP;
    int c_move = 0;
    int c_rotate = 0;
    int b_rotate = 0;
    int c_conv_speed = 0;
    int t_conv_speed = 0;
    int b_throw_speed = 0;
    int box_catch = 0;

    bool is_t_conv = false;
    auto box_status = c_state::STOP;

    CAN can(PA_11, PA_12, 1000000);
    CANMessage msg_encoder;
    BufferedSerial pc(USBTX, USBRX, 115200);
    // BufferedSerial controller(PA_9, PA_10, 115200);
    dji::C620 c620(PB_12, PB_13);

    PidGain pid_gain = {0.001, 0.00001, 0.0};
    std::array<Pid, motor_amount> pid = {Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max})};

    for (int i = 0; i < motor_amount; i++)
    {
        pid[i].reset();
    }

    c620.set_max_output(dji_max_output);
    ryugu = 0;

    while (true)
    {
        auto now = HighResClock::now();
        static auto pre = now;

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

        char data[10] = "";
        if (readline(pc, data) == 0)
        {
            if (strcmp(data, "vel") == 0)
            {
                for (int i = 0; i < motor_amount; i++)
                {
                    char data_vel[10] = "";
                    if (readline(pc, data_vel, false, true) == 0)
                    {
                        motor_velocity[i] = atoi(data_vel) * 19 * -1;
                    }
                }
            }

            if (strcmp(data, "sort_t") == 0)
            {
                servo1[0] = 112;
            }
            else if (strcmp(data, "sort_s") == 0)
            {
                servo1[0] = 144;
            }
            if (strcmp(data, "b_conv") == 0)
            {
                is_t_conv = !is_t_conv;
            }

            if (strcmp(data, "b_launch") == 0)
            {
                is_b_throw = true
            }
            else if (strcmp(data, "b_launch_s") == 0)
            {
                is_b_throw = false;
            }
            if (strcmp(data, "k_up") == 0)
            {
                box_status = c_state::UP;
            }
            else if (strcmp(data, "k_down") == 0)
            {
                box_status = c_state::DOWN;
            }
            else if (strcmp(data, "k_stop") == 0)
            {
                box_status = c_state::STOP;
            }

            if (strcmp(data, "c_indo") == 0)
            {
                //    printf("Pushed Circle\n");
                is_c_indoroll = !is_c_indoroll;
            }
            if (strcmp(data, "b_indo") == 0)
            {
                //    printf("Pushed Square\n");
                is_b_indoroll = !is_b_indoroll;
            }
            if (strcmp(data, "c_up") == 0)
            {
                c_direction = c_state::UP;
            }
            if (strcmp(data, "c_down") == 0)
            {
                c_direction = c_state::DOWN;
            }
            if (strcmp(data, "c_stop") == 0)
            {
                c_direction = c_state::STOP;
            }
            if (strcmp(data, "c_push") == 0) //TODO: neo_pushに変更s
            {
                ryugu = !ryugu;
            }


            if (c_direction == c_state::UP)
            {
                c_move = 10000;
            }
            else if (c_direction == c_state::DOWN)
            {
                c_move = -10000;
            }
            else
            {
                c_move = 0;
            }

            if (is_c_indoroll)
            {
                c_rotate = 8000;
            }
            else
            {
                c_rotate = 0;
            }

            if (is_b_indoroll)
            {
                b_rotate = 8000;
            }
            else
            {
                b_rotate = 0;
            }

            if (c_move_belt)
            {
                c_conv_speed = 10000;
            }
            else
            {
                c_conv_speed = 0;
            }
            if (is_t_conv)
            {
                t_conv_speed = 16000;
            }
            else
            {
                t_conv_speed = 0;
            }
            if (is_b_throw)
            {
                b_throw_speed = 13000;
            }
            else
            {
                b_throw_speed = 0;
            }
            if (box_status == c_state::UP)
            {
                box_catch = 15000;
            }
            else if (box_status == c_state::DOWN)
            {
                box_catch = -15000;
            }
            else
            {
                box_catch = 0;
            }

            pwm1[0] = b_throw_speed;
            pwm1[1] = b_rotate;
            pwm1[2] = t_conv_speed;
            pwm1[3] = c_rotate;
            pwm2[0] = 0;
            pwm2[1] = c_move;
            pwm2[2] = box_catch;
            pwm2[3] = c_conv_speed;
        }
        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("encoder_data: %d, %d, %d, %d\n", encoder_data[0], encoder_data[1], encoder_data[2], encoder_data[3]);

            int16_t motor_output[motor_amount] = {0};

            for (int i = 0; i < motor_amount; i++)
            {
                float rmp_to_rad = 2 * M_PI / 60;
                float motor_dps = c620.get_rpm(i + 1) * rmp_to_rad;
                int goal_ang_vel = motor_velocity[i];
                const float percent = pid[i].calc(goal_ang_vel, motor_dps, elapsed);
                // printf("dps: %f, goal: %d, out: %d\n", motor_dps, goal_ang_vel, out);

                c620.set_output_percent(percent, i + 1);
            }

            for (int i = 0; i < motor_amount; i++)
            {
                motor_output[i] = c620.get_current(i + 1);
            }
            printf("motor_output: %d, %d, %d, %d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            // printf("motor_dps: %d, %d, %d, %d\n", motor_dps[0], motor_dps[1], motor_dps[2], motor_dps[3]);
            c620.write();
            
            CANMessage msg1(CAN_ID1, (const uint8_t *)&pwm1, 8);
            CANMessage msg2(CAN_ID2, (const uint8_t *)&pwm2, 8);
            CANMessage msg_servo(CAN_ID_SERVO, (const uint8_t *)&servo1, 8);
            can1.write(msg1);
            can1.write(msg2);
            can1.write(msg_servo);

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

    while ((buff != '\n') and i < 10)
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

