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
    constexpr int pid_max = 1;
    constexpr int dji_max_output = 8000;
    constexpr int motor_amount = 4;
    int velocity_xy[2] = {0}; // x, y, ang
    float velocity_ang = 0;
    int pos_mm[3] = {0}; // x, y, ang
    int motor_velocity[motor_amount] = {0};

    constexpr int wheel_radius = 34;     // mm
    constexpr int robot_size_calc = 190; // mm
    constexpr int robot_size_motor = 240;

    constexpr int max_dps_trans = 20000;
    constexpr int max_dps_rot = 500;

    int encoder_data[motor_amount] = {0};
    int encoder_data_pre[motor_amount] = {0};

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
            printf("enc_data\n%d\n%d\n%d\n%d\n", encoder_data[0], encoder_data[1], encoder_data[2], encoder_data[3]);
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
                const int out = percent * dji_max_output;
                printf("dps: %f, goal: %d, out: %d\n", motor_dps, goal_ang_vel, out);

                c620.set_output(out, i + 1);
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