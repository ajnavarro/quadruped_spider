#ifndef spider2_h
#define spider2_h

#include <Arduino.h>
#include <Servo9685.h>

#define KEEP 255

enum COMMAND
{
    IDLE,
    STAND,
    SIT,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

class SpiderV2
{
public:
    SpiderV2();
    void init();
    void loop();
    void set_speed(float speed);
    void sit();
    void stand();
    void stop();
    void forward();
    void backward();
    void left();
    void right();
    void align();

    /* State control ------------------------------------------------------------ */
    int step;
    COMMAND actual_command = IDLE;
    COMMAND next_command = IDLE;

private:
    /* Pwm interface pins and controller ---------------------------------------- */
    servo_PCA9685 *pwm;
    Servo9685 servo[4][3];
    const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
    /* Size of the robot ---------------------------------------------------------*/
    const float length_a = 55;
    const float length_b = 77.5;
    const float length_c = 27.5;
    const float length_side = 69.6;
    const float z_absolute = -31.6;
    /* Constants for movement ----------------------------------------------------*/
    const float z_default = -55, z_up = -35, z_boot = z_absolute;
    const float x_default = 60;
    const float y_start = 0, y_step = 45;
    const float y_default = x_default;
    /* variables for movement ----------------------------------------------------*/
    volatile float site_now[4][3];    //real-time coordinates of the end of each leg
    volatile float site_expect[4][3]; //expected coordinates of the end of each leg
    float temp_speed[4][3];           //each axis' speed, needs to be recalculated before each movement
    float move_speed;                 //movement speed
    /* Constants for turn --------------------------------------------------------*/
    //temp length
    const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
    const float temp_b = 2 * (y_start + y_step) + length_side;
    const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
    const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
    //site for turn
    const float turn_x1 = (temp_a - length_side) / 2;
    const float turn_y1 = y_start + y_step / 2;
    const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
    const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

    /* Time variables to handle correctly servo updates on loop ----------------- */
    const unsigned int period = 20; //20 ms
    unsigned long time_now = 0;

    /* Functions ---------------------------------------------------------------- */
    void servo_attach();
    void set_site(int leg, float x, float y, float z);

    bool reached(int leg);
    void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
    void polar_to_servo(int leg, float alpha, float beta, float gamma);

    bool all_reached();
    bool handle_z(float z);
    bool handle_forward();
    bool handle_backward();
    bool handle_left();
    bool handle_right();

    void set_command(COMMAND c);
};

void SpiderV2::set_command(COMMAND c)
{
    if (actual_command == IDLE)
    {
        step = 0;
        if (next_command == IDLE)
        {
            actual_command = c;
        }
        else
        {
            actual_command = next_command;
        }
    }

    next_command = c;
}

void SpiderV2::align()
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            servo[i][j].write(90);
        }
    }
}

void SpiderV2::sit()
{
    set_command(SIT);
}

void SpiderV2::stand()
{
    set_command(STAND);
}

void SpiderV2::stop()
{
    set_command(IDLE);
}

void SpiderV2::forward()
{
    set_command(FORWARD);
}

void SpiderV2::backward()
{
    set_command(BACKWARD);
}

void SpiderV2::left()
{
    set_command(LEFT);
}

void SpiderV2::right()
{
    set_command(RIGHT);
}

bool SpiderV2::handle_forward()
{
    switch (step)
    {
    case 0:
        set_speed(8);
        if (!site_now[2][1] == y_start)
        {
            step = 8;
        }
        break;
    case 1:
        set_site(2, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 2:
        set_site(2, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 3:
        set_site(2, x_default, y_start + 2 * y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 4:
        set_speed(3);
        set_site(0, x_default, y_start, z_default);
        set_site(1, x_default, y_start + 2 * y_step, z_default);
        set_site(2, x_default, y_start + y_step, z_default);
        set_site(3, x_default, y_start + y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 5:
        set_speed(8);
        set_site(1, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 6:
        set_site(1, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 7:
        set_site(1, x_default, y_start, z_default);
        if (all_reached())
        {
            step++;
            return true;
        }
        break;
    case 8:
        set_site(0, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 9:
        set_site(0, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 10:
        set_site(0, x_default, y_start + 2 * y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 11:
        set_speed(3);
        set_site(0, x_default, y_start + y_step, z_default);
        set_site(1, x_default, y_start + y_step, z_default);
        set_site(2, x_default, y_start, z_default);
        set_site(3, x_default, y_start + 2 * y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 12:
        set_speed(8);

        set_site(3, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 13:
        set_site(3, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 14:
        set_site(3, x_default, y_start, z_default);
        if (!all_reached())
        {
            step = 0;
            return true;
        }
        break;
    }

    step++;
    return false;
}

bool SpiderV2::handle_backward()
{
    switch (step)
    {
    case 0:
        set_speed(8);
        if (!site_now[3][1] == y_start)
        {
            step = 8;
        }
        break;
    case 1:
        set_site(3, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 2:
        set_site(3, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 3:
        set_site(3, x_default, y_start + 2 * y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 4:
        set_speed(3);
        set_site(0, x_default, y_start + 2 * y_step, z_default);
        set_site(1, x_default, y_start, z_default);
        set_site(2, x_default, y_start + y_step, z_default);
        set_site(3, x_default, y_start + y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 5:
        set_speed(8);
        set_site(0, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 6:
        set_site(0, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 7:
        set_site(0, x_default, y_start, z_default);
        if (all_reached())
        {
            step++;
            return true;
        }
        break;
    case 8:
        set_site(1, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 9:
        set_site(1, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 10:
        set_site(1, x_default, y_start + 2 * y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 11:
        set_speed(3);
        set_site(0, x_default, y_start + y_step, z_default);
        set_site(1, x_default, y_start + y_step, z_default);
        set_site(2, x_default, y_start + 2 * y_step, z_default);
        set_site(3, x_default, y_start, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 12:
        set_speed(8);

        set_site(2, x_default, y_start + 2 * y_step, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 13:
        set_site(2, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 14:
        set_site(2, x_default, y_start, z_default);
        if (!all_reached())
        {
            step = 0;
            return true;
        }
        break;
    }

    step++;
    return false;
}

bool SpiderV2::handle_right()
{
    switch (step)
    {
    case 0:
        set_speed(4);
        if (!site_now[2][1] == y_start)
        {
            step = 8;
        }
        break;
    case 1:
        set_site(2, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 2:
        set_site(0, turn_x0, turn_y0, z_default);
        set_site(1, turn_x1, turn_y1, z_default);
        set_site(2, turn_x0, turn_y0, z_up);
        set_site(3, turn_x1, turn_y1, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 3:
        set_site(2, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 4:
        set_site(0, turn_x0, turn_y0, z_default);
        set_site(1, turn_x1, turn_y1, z_default);
        set_site(2, turn_x0, turn_y0, z_default);
        set_site(3, turn_x1, turn_y1, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 5:
        set_site(0, turn_x0, turn_y0, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 6:
        set_site(0, x_default, y_start, z_up);
        set_site(1, x_default, y_start, z_default);
        set_site(2, x_default, y_start + y_step, z_default);
        set_site(3, x_default, y_start + y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 7:
        set_site(0, x_default, y_start, z_default);
        if (all_reached())
        {
            step++;
            return true;
        }
        break;
    case 8:
        set_site(1, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 9:
        set_site(0, turn_x1, turn_y1, z_default);
        set_site(1, turn_x0, turn_y0, z_up);
        set_site(2, turn_x1, turn_y1, z_default);
        set_site(3, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 10:
        set_site(1, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 11:
        set_site(0, turn_x1, turn_y1, z_default);
        set_site(1, turn_x0, turn_y0, z_default);
        set_site(2, turn_x1, turn_y1, z_default);
        set_site(3, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 12:
        set_site(3, turn_x0, turn_y0, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 13:
        set_site(0, x_default, y_start + y_step, z_default);
        set_site(1, x_default, y_start + y_step, z_default);
        set_site(2, x_default, y_start, z_default);
        set_site(3, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;

    case 14:
        set_site(3, x_default, y_start, z_default);
        if (!all_reached())
        {
            step = 0;
            return true;
        }
        break;
    }

    step++;
    return false;
}

bool SpiderV2::handle_left()
{
    switch (step)
    {
    case 0:
        set_speed(4);
        if (!site_now[3][1] == y_start)
        {
            step = 8;
        }
        break;
    case 1:
        set_site(3, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 2:
        set_site(0, turn_x1, turn_y1, z_default);
        set_site(1, turn_x0, turn_y0, z_default);
        set_site(2, turn_x1, turn_y1, z_default);
        set_site(3, turn_x0, turn_y0, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 3:
        set_site(3, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 4:
        set_site(0, turn_x1, turn_y1, z_default);
        set_site(1, turn_x0, turn_y0, z_default);
        set_site(2, turn_x1, turn_y1, z_default);
        set_site(3, turn_x0, turn_y0, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 5:
        set_site(1, turn_x0, turn_y0, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 6:
        set_site(0, x_default, y_start, z_default);
        set_site(1, x_default, y_start, z_up);
        set_site(2, x_default, y_start + y_step, z_default);
        set_site(3, x_default, y_start + y_step, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 7:
        set_site(1, x_default, y_start, z_default);
        if (all_reached())
        {
            step++;
            return true;
        }
        break;
    case 8:
        set_site(0, x_default, y_start, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 9:
        set_site(0, turn_x0, turn_y0, z_up);
        set_site(1, turn_x1, turn_y1, z_default);
        set_site(2, turn_x0, turn_y0, z_default);
        set_site(3, turn_x1, turn_y1, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 10:
        set_site(0, turn_x0, turn_y0, z_default);
        set_site(1, turn_x1, turn_y1, z_default);
        set_site(2, turn_x0, turn_y0, z_default);
        set_site(3, turn_x1, turn_y1, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 11:
        set_site(2, turn_x0, turn_y0, z_up);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 12:
        set_site(0, x_default, y_start + y_step, z_default);
        set_site(1, x_default, y_start + y_step, z_default);
        set_site(2, x_default, y_start, z_up);
        set_site(3, x_default, y_start, z_default);
        if (!all_reached())
        {
            return false;
        }
        break;
    case 13:
        set_site(2, x_default, y_start, z_default);
        if (!all_reached())
        {
            step = 0;
            return true;
        }
        break;
    }

    step++;
    return false;
}

bool SpiderV2::handle_z(float z)
{
    switch (step)
    {
    case 0:
        set_speed(1);
        for (int leg = 0; leg < 4; leg++)
        {
            set_site(leg, KEEP, KEEP, z);
        }
        break;
    case 1:
        return all_reached();
    }

    step++;
    return false;
};

void SpiderV2::loop()
{
    if (millis() - time_now < period)
    {
        return;
    }

    time_now = millis();

    static float alpha, beta, gamma;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (abs(site_now[i][j] - site_expect[i][j]) > abs(temp_speed[i][j]))
                site_now[i][j] += temp_speed[i][j];
            else
                site_now[i][j] = site_expect[i][j];
        }

        cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
        polar_to_servo(i, alpha, beta, gamma);
    }

    bool finished = false;
    switch (actual_command)
    {
    case STAND:
        finished = handle_z(z_default);
        break;
    case SIT:
        finished = handle_z(z_boot);
        break;
    case FORWARD:
        finished = handle_forward();
        break;
    case BACKWARD:
        finished = handle_backward();
        break;
    case LEFT:
        finished = handle_left();
        break;
    case RIGHT:
        finished = handle_right();
        break;
    case IDLE:
        break;
    }

    if (finished)
    {
        step = 0;
        actual_command = next_command;
        next_command = IDLE;
    }
}

void SpiderV2::cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
    //calculate w-z degree
    float v, w;
    w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
    v = w - length_c;
    alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
    beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
    //calculate x-y-z degree
    gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
    //trans degree pi->180
    alpha = alpha / PI * 180;
    beta = beta / PI * 180;
    gamma = gamma / PI * 180;
}

void SpiderV2::polar_to_servo(int leg, float alpha, float beta, float gamma)
{
    if (leg == 0 || leg == 3)
    {
        alpha = 90 - alpha;
        beta = beta;
        gamma += 90;
    }
    else if (leg == 1 || leg == 2)
    {
        alpha += 90;
        beta = 180 - beta;
        gamma = 90 - gamma;
    }

    servo[leg][0].write(alpha);
    servo[leg][1].write(beta);
    servo[leg][2].write(gamma);
}

void SpiderV2::set_speed(float speed)
{
    move_speed = speed;
}

bool SpiderV2::all_reached(void)
{
    return reached(0) &&
           reached(1) &&
           reached(2) &&
           reached(3);
}

bool SpiderV2::reached(int leg)
{
    return (site_now[leg][0] == site_expect[leg][0]) &&
           (site_now[leg][1] == site_expect[leg][1]) &&
           (site_now[leg][2] == site_expect[leg][2]);
}

void SpiderV2::set_site(int leg, float x, float y, float z)
{
    float length_x = 0, length_y = 0, length_z = 0;

    if (x != KEEP)
        length_x = x - site_now[leg][0];
    if (y != KEEP)
        length_y = y - site_now[leg][1];
    if (z != KEEP)
        length_z = z - site_now[leg][2];

    float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

    temp_speed[leg][0] = length_x / length * move_speed;
    temp_speed[leg][1] = length_y / length * move_speed;
    temp_speed[leg][2] = length_z / length * move_speed;

    if (x != KEEP)
    {
        site_expect[leg][0] = x;
    }
    if (y != KEEP)
    {
        site_expect[leg][1] = y;
    }
    if (z != KEEP)
    {
        site_expect[leg][2] = z;
    }
}

void SpiderV2::servo_attach(void)
{
    pwm = new servo_PCA9685();
    pwm->begin();
    pwm->setPWMFreq(50);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            servo[i][j].attach(servo_pin[i][j], pwm, 100, 500);
        }
    }
}

SpiderV2::SpiderV2()
{
    //initialize default parameter
    set_site(0, x_default, y_start + y_step, z_boot);
    set_site(1, x_default, y_start + y_step, z_boot);
    set_site(2, x_default, y_start, z_boot);
    set_site(3, x_default, y_start, z_boot);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            site_now[i][j] = site_expect[i][j];
        }
    }
    servo_attach();
}

#endif