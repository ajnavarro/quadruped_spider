#include "Spider.h"

Spider::Spider()
{
    //initialize default parameter
    set_site(0, x_default - x_offset, y_start + y_step, z_boot);
    set_site(1, x_default - x_offset, y_start + y_step, z_boot);
    set_site(2, x_default + x_offset, y_start, z_boot);
    set_site(3, x_default + x_offset, y_start, z_boot);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            site_now[i][j] = site_expect[i][j];
        }
    }
    servo_attach();
}

void Spider::move_z(float z_pos)
{
    move_speed = stand_seat_speed;
    for (int leg = 0; leg < 4; leg++)
    {
        set_site(leg, KEEP, KEEP, z_pos);
    }
    wait_all_reach();
}

void Spider::turn_left(int steps)
{
    move_speed = spot_turn_speed;
    while (steps-- > 0)
    {
        if (site_now[3][1] == y_start)
        {
            //leg 3&1 move
            set_site(3, x_default + x_offset, y_start, z_up);
            wait_all_reach();

            set_site(0, turn_x1 - x_offset, turn_y1, z_default);
            set_site(1, turn_x0 - x_offset, turn_y0, z_default);
            set_site(2, turn_x1 + x_offset, turn_y1, z_default);
            set_site(3, turn_x0 + x_offset, turn_y0, z_up);
            wait_all_reach();

            set_site(3, turn_x0 + x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(0, turn_x1 + x_offset, turn_y1, z_default);
            set_site(1, turn_x0 + x_offset, turn_y0, z_default);
            set_site(2, turn_x1 - x_offset, turn_y1, z_default);
            set_site(3, turn_x0 - x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(1, turn_x0 + x_offset, turn_y0, z_up);
            wait_all_reach();

            set_site(0, x_default + x_offset, y_start, z_default);
            set_site(1, x_default + x_offset, y_start, z_up);
            set_site(2, x_default - x_offset, y_start + y_step, z_default);
            set_site(3, x_default - x_offset, y_start + y_step, z_default);
            wait_all_reach();

            set_site(1, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
        else
        {
            //leg 0&2 move
            set_site(0, x_default + x_offset, y_start, z_up);
            wait_all_reach();

            set_site(0, turn_x0 + x_offset, turn_y0, z_up);
            set_site(1, turn_x1 + x_offset, turn_y1, z_default);
            set_site(2, turn_x0 - x_offset, turn_y0, z_default);
            set_site(3, turn_x1 - x_offset, turn_y1, z_default);
            set_site(0, turn_x0 + x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(0, turn_x0 - x_offset, turn_y0, z_default);
            set_site(1, turn_x1 - x_offset, turn_y1, z_default);
            set_site(2, turn_x0 + x_offset, turn_y0, z_default);
            set_site(3, turn_x1 + x_offset, turn_y1, z_default);
            wait_all_reach();

            set_site(2, turn_x0 + x_offset, turn_y0, z_up);
            wait_all_reach();

            set_site(0, x_default - x_offset, y_start + y_step, z_default);
            set_site(1, x_default - x_offset, y_start + y_step, z_default);
            set_site(2, x_default + x_offset, y_start, z_up);
            set_site(3, x_default + x_offset, y_start, z_default);
            wait_all_reach();

            set_site(2, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
    }
}
void Spider::turn_right(int steps)
{
    move_speed = spot_turn_speed;
    while (steps-- > 0)
    {
        if (site_now[2][1] == y_start)
        {
            //leg 2&0 move
            set_site(2, x_default + x_offset, y_start, z_up);
            wait_all_reach();

            set_site(0, turn_x0 - x_offset, turn_y0, z_default);
            set_site(1, turn_x1 - x_offset, turn_y1, z_default);
            set_site(2, turn_x0 + x_offset, turn_y0, z_up);
            set_site(3, turn_x1 + x_offset, turn_y1, z_default);
            wait_all_reach();

            set_site(2, turn_x0 + x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(0, turn_x0 + x_offset, turn_y0, z_default);
            set_site(1, turn_x1 + x_offset, turn_y1, z_default);
            set_site(2, turn_x0 - x_offset, turn_y0, z_default);
            set_site(3, turn_x1 - x_offset, turn_y1, z_default);
            wait_all_reach();

            set_site(0, turn_x0 + x_offset, turn_y0, z_up);
            wait_all_reach();

            set_site(0, x_default + x_offset, y_start, z_up);
            set_site(1, x_default + x_offset, y_start, z_default);
            set_site(2, x_default - x_offset, y_start + y_step, z_default);
            set_site(3, x_default - x_offset, y_start + y_step, z_default);
            wait_all_reach();

            set_site(0, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
        else
        {
            //leg 1&3 move
            set_site(1, x_default + x_offset, y_start, z_up);
            wait_all_reach();

            set_site(0, turn_x1 + x_offset, turn_y1, z_default);
            set_site(1, turn_x0 + x_offset, turn_y0, z_up);
            set_site(2, turn_x1 - x_offset, turn_y1, z_default);
            set_site(3, turn_x0 - x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(1, turn_x0 + x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(0, turn_x1 - x_offset, turn_y1, z_default);
            set_site(1, turn_x0 - x_offset, turn_y0, z_default);
            set_site(2, turn_x1 + x_offset, turn_y1, z_default);
            set_site(3, turn_x0 + x_offset, turn_y0, z_default);
            wait_all_reach();

            set_site(3, turn_x0 + x_offset, turn_y0, z_up);
            wait_all_reach();

            set_site(0, x_default - x_offset, y_start + y_step, z_default);
            set_site(1, x_default - x_offset, y_start + y_step, z_default);
            set_site(2, x_default + x_offset, y_start, z_default);
            set_site(3, x_default + x_offset, y_start, z_up);
            wait_all_reach();

            set_site(3, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
    }
}
void Spider::step_forward(int steps)
{
    move_speed = leg_move_speed;
    while (steps-- > 0)
    {
        if (site_now[2][1] == y_start)
        {
            //leg 2&1 move
            set_site(2, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
            wait_all_reach();

            move_speed = body_move_speed;

            set_site(0, x_default + x_offset, y_start, z_default);
            set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
            set_site(2, x_default - x_offset, y_start + y_step, z_default);
            set_site(3, x_default - x_offset, y_start + y_step, z_default);
            wait_all_reach();

            move_speed = leg_move_speed;

            set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(1, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(1, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
        else
        {
            //leg 0&3 move
            set_site(0, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
            wait_all_reach();

            move_speed = body_move_speed;

            set_site(0, x_default - x_offset, y_start + y_step, z_default);
            set_site(1, x_default - x_offset, y_start + y_step, z_default);
            set_site(2, x_default + x_offset, y_start, z_default);
            set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
            wait_all_reach();

            move_speed = leg_move_speed;

            set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(3, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(3, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
    }
}

void Spider::step_backwards(int steps)
{
    move_speed = leg_move_speed;
    while (steps-- > 0)
    {
        if (site_now[3][1] == y_start)
        {
            //leg 3&0 move
            set_site(3, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
            wait_all_reach();

            move_speed = body_move_speed;

            set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
            set_site(1, x_default + x_offset, y_start, z_default);
            set_site(2, x_default - x_offset, y_start + y_step, z_default);
            set_site(3, x_default - x_offset, y_start + y_step, z_default);
            wait_all_reach();

            move_speed = leg_move_speed;

            set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(0, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(0, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
        else
        {
            //leg 1&2 move
            set_site(1, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
            wait_all_reach();

            move_speed = body_move_speed;

            set_site(0, x_default - x_offset, y_start + y_step, z_default);
            set_site(1, x_default - x_offset, y_start + y_step, z_default);
            set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
            set_site(3, x_default + x_offset, y_start, z_default);
            wait_all_reach();

            move_speed = leg_move_speed;

            set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
            wait_all_reach();
            set_site(2, x_default + x_offset, y_start, z_up);
            wait_all_reach();
            set_site(2, x_default + x_offset, y_start, z_default);
            wait_all_reach();
        }
    }
}

void Spider::body_move(long x, long y, long z)
{
    move_speed = stand_seat_speed;

    long x1 = map(x, -10, 10, x_default - 30, x_default + 30);
    long x2 = map(x, -10, 10, x_default + 30, x_default - 30);

    long y1 = map(y, -10, 10, y_default + 30, y_default - 30);
    long y2 = map(y, -10, 10, y_default - 30, y_default + 30);

    long z1 = map(z, -10, 10, z_default - 20, z_default + 20);
    long z2 = map(z, -10, 10, z_default + 20, z_default - 20);

    set_site(0, x1, y1, z1);
    set_site(1, x1, y2, z2);
    set_site(2, x2, y1, z1);
    set_site(3, x2, y2, z2);

    wait_all_reach();
}

void Spider::align()
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            servo[i][j].write(90);
            delay(20);
        }
    }
}

void Spider::set_site(int leg, float x, float y, float z)
{
    float length_x = 0, length_y = 0, length_z = 0;

    if (x != KEEP)
        length_x = x - site_now[leg][0];
    if (y != KEEP)
        length_y = y - site_now[leg][1];
    if (z != KEEP)
        length_z = z - site_now[leg][2];

    float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

    temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
    temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
    temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

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

void Spider::wait_reach(int leg)
{
    while (1)
    {
        if (site_now[leg][0] == site_expect[leg][0])
            if (site_now[leg][1] == site_expect[leg][1])
                if (site_now[leg][2] == site_expect[leg][2])
                    break;
        yield();
    }
}

void Spider::wait_all_reach(void)
{
    for (int i = 0; i < 4; i++)
        wait_reach(i);
}

void Spider::servo_attach(void)
{
    pwm = new servo_PCA9685();
    pwm->begin();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            servo[i][j].attach(servo_pin[i][j], pwm, 100, 500);
        }
    }
}

void Spider::servo_service()
{
    sei();
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
}

void Spider::cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
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

void Spider::polar_to_servo(int leg, float alpha, float beta, float gamma)
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