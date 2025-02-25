#include <mbed.h>

/// @Tips mm rad s
#include <PID_new.hpp>
#include <cmath>
#include <array>
#include <C620.hpp>

bool readline(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);

int main()
{   
    CAN can1(PA_11, PA_12, (int)1e6);
    int16_t pwm1[4] = {0, 0, 0, 0};    // pwm配列,zassou
    int16_t pwm2[4] = {0, 0, 0, 0};    // pwm配列,nyayuta
    int8_t servo1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    CANMessage msg1;
    CANMessage msg2;
    int CAN_ID = 2;
    int CAN_ID2 = 3;

    constexpr int pid_max = 1;
    constexpr int dji_max_output = 8000;
    constexpr int motor_amount = 4;
    int velocity_xy[2] = {0}; // x, y, ang
    float velocity_ang = 0;
    int pos_mm[3] = {0}; // x, y, ang
    int motor_velocity[motor_amount] = {0};
    constexpr int wheel_radius = 50; // mm
    constexpr int robot_size = 300;  // mm
    constexpr int max_dps_trans = 20000;
    constexpr int max_dps_rot = 500;

    bool is_c_indoroll = false;
    bool is_b_indoroll = false;
    bool pushed_L1 = false;
    bool pushed_R1 = false;
    bool c_move_belt = false;
    float c_move = 0;
    float c_rotate = 0;
    float b_rotate = 0;
    float c_conv_speed = 0;
    constexpr bool is_calc = false;

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

    while (true)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        for (int i = 0; i < motor_amount; i++)
        {
            c620.read_data();
        }

        char data[10] = "";
        if (readline(pc, data) == 0)
        {
            // printf("%s\n", data);

            if (strcmp(data, "trans") == 0)
            {
                // printf("trans!!\n");
                for (int i = 0; i < 2; i++)
                {
                    char data_trans[10] = "";
                    if (readline(pc, data_trans, false, true) == 0)
                    {
                        velocity_xy[i] = atof(data_trans) * max_dps_trans * -1;
                    }
                }
            }
            if (strcmp(data, "rot") == 0)
            {
                char data_rot[10] = "";
                if (readline(pc, data_rot, false, true) == 0)
                {
                    velocity_ang = atof(data_rot) * max_dps_rot * -1;
                    // printf("velocity_ang: %f\n", velocity_ang);
                }
            }
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
            if (strcmp(data, "c_indo") == 0){
            //    printf("Pushed Circle\n");
                if(is_c_indoroll){
                    is_c_indoroll = false;
                }else{
                    is_c_indoroll = true;
                }
            }
            if (strcmp(data, "b_indo") == 0){
            //    printf("Pushed Square\n");
                if(is_b_indoroll){
                    is_b_indoroll = false;
                }else{
                    is_b_indoroll = true;
                }
            }
            if (strcmp(data, "c_up") == 0){
                pushed_L1 = true;
                pushed_R1 = false;
            }
            if (strcmp(data, "c_down") == 0){
                pushed_L1 = false;
                pushed_R1 = true;
            }
            if (strcmp(data, "c_stop") == 0){
                pushed_L1 = false;
                pushed_R1 = false;
            }
            if (strcmp(data, "c_conv") == 0){
                if(c_move_belt){
                    c_move_belt = false;
                }else{
                    c_move_belt = true;
                }
            }
        }
        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("encoder_diff: %d, %d, %d, %d\n", encoder_diff[0], encoder_diff[1], encoder_diff[2], encoder_diff[3]);

             int16_t motor_output[motor_amount] = {0};
            if (is_calc)
            {
                float theta_rad = atan2(velocity_xy[1], velocity_xy[0]);
                float output_power = hypot(velocity_xy[0], velocity_xy[1]);
                // printf("output_power: %f\n", output_power);
                for (int i = 0; i < motor_amount; i++)
                {
                    float rmp_to_rad = 2 * M_PI / 60;
                    float motor_dps = c620.get_rpm(i + 1) * rmp_to_rad;
                    float motor_ang = i * M_PI / 2;
                    float motor_offset = M_PI / 4;
                    float goal_ang_vel = (sin(theta_rad - (motor_ang + motor_offset)) * output_power + velocity_ang * robot_size) / wheel_radius;
                    // printf("goal_ang_vel %d: %f, dps: %f\n", i, goal_ang_vel, motor_dps);
                    const float out = pid[i].calc(goal_ang_vel, motor_dps, elapsed);
                    printf("out: %f\n", out);
                    c620.set_output_percent(out, i + 1);
                    // c620.set_output(0, i+1);
                }
            }
            else
            {
                for (int i = 0; i < motor_amount; i++)
                {
                    float rmp_to_rad = 2 * M_PI / 60;
                    float motor_dps = c620.get_rpm(i + 1) * rmp_to_rad;
                    int goal_ang_vel = motor_velocity[i];
                    const float percent = pid[i].calc(goal_ang_vel, motor_dps, elapsed);
                    const int out = percent * dji_max_output;
                    printf("dps: %f, goal: %d, out: %d\n", motor_dps, goal_ang_vel, out);

                    c620.set_output(out, i + 1);
                }
            }
            for (int i = 0; i < motor_amount; i++)
            {
                motor_output[i] = c620.get_current(i + 1);
            }
            printf("motor_output: %d, %d, %d, %d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            // printf("motor_dps: %d, %d, %d, %d\n", motor_dps[0], motor_dps[1], motor_dps[2], motor_dps[3]);
            c620.write();
            pre = now;
        }

        if(pushed_L1){
            c_move = 10000;
        }else if(pushed_R1){
            c_move = -10000;
        }else{
            c_move = 0;
        }

        if(is_c_indoroll){
            c_rotate = 8000;
        }else{
            c_rotate = 0;
        }

        if(is_b_indoroll){
            b_rotate = 8000;
        }else{
            b_rotate = 0;
        }

        if(c_move_belt){
            c_conv_speed = 10000;
        }else{
            c_conv_speed = 0;
        }

        pwm2 = {c_rotate, b_rotate, c_move, c_conv_speed};

        CANMessage msg2(CAN_ID2, (const uint8_t *)&pwm2, 8);
        can1.write(msg2);
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