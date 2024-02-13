#ifndef STRUCTS_HPP
#define STRUCTS_HPP


/* 
大きく４つの構造体グループに分けて参照渡しする。
1. センサー関連
2. マウス動作関連
3. 制御関連
4. マップ関連
*/

typedef enum
{
    FALSE = 0,
    TRUE = 1,
}t_bool;

typedef enum
{
    FRONT = 0,
    RIGHT = 1,
    REAR = 2,
    LEFT = 3,
    UNDEFINED,
}t_local_dir;

typedef enum
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
}t_direction;

typedef enum
{
    NOWALL = 0,
    WALL = 1,
    UNKNOWN = 2,
}t_exist_wall;

typedef struct 
{
    int f = 0;  //front
    int fl = 0; //front left
    int fr = 0; //front right
    int l = 0;  //left
    int r = 0;  //right
    int b = 0;  //back
}t_sens_dir;    //sensor direction data

typedef struct 
{
    t_bool l = FALSE; //front
    t_bool fl = FALSE;    //front left
    t_bool fr = FALSE;    //front right
    t_bool r = FALSE; //left
}t_wall_exist;  //wall exist data

typedef struct 
{
    t_sens_dir val;  //sensor value
    t_sens_dir d_val;    //sensor value difference
    t_sens_dir p_val;    //sensor value past
    t_sens_dir error;    //sensor value error
    t_sens_dir ref;  //sensor value reference
    t_sens_dir th_wall;  //wall threshold value
    t_sens_dir th_control;   //control threshold value
    t_wall_exist exist; //wall true or false
    t_wall_exist control_enable;  //control true or false
    t_bool control;  //enable or disable
    t_sens_dir centor_front;  //center value
    t_sens_dir center_right;
    t_sens_dir center_left;
    t_sens_dir center_rear;
}t_wall_sens;  //wall sensor data

typedef struct 
{
    float yaw = 0; //gyro yaw
    float yaw_new = 0; //gyro yaw new
    float ref = 0; //gyro reference
    float degree = 0;
    float radian = 0;
}t_gyro;    //gyro data

typedef struct 
{
    unsigned int angle = 0;
    t_sens_dir data;
    t_sens_dir locate;
    t_sens_dir p_locate;
    t_sens_dir diff_pulse;
    t_sens_dir diff_p_pulse;    
}t_enc;     //encoder data

typedef struct 
{
    t_wall_sens wall;
    t_gyro gyro;
    t_enc enc;
    float BatteryVoltage = 0;
}t_sens_data;   //sensor data

typedef struct
{
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
    uint16_t Angle = 0;
    uint16_t Angle_R = 0;
    uint16_t Angle_L = 0;
    float BatteryVoltage = 0;
    int sens_r_value = 0;
    int sens_l_value = 0;
    int sens_fr_value = 0;
    int sens_fl_value = 0;
}t_sensing_result;



typedef struct 
{
    float vel = 0;  //velocity
    float ang_vel = 0;  //angular velocity
    float deg = 0;  //degree
    float rad = 0;  //radian
    float vel_error = 0; //error
    float ang_error = 0;    //angular error
    float acc = 0;  //acceleration
    float ang_acc = 0;  //angular acceleration
    float len = 0;   //length
    float len_half = 0; //length half
    float wall_val = 0; //wall value
    float wall_error = 0;   //wall error
    float alpha = 0;    //相補フィルタ用
    t_local_dir flag;
}t_motion;  //motion parameter



typedef struct 
{
    t_motion r;
    t_motion l;
    t_motion p;    //past
    t_motion current;   //current
    t_motion max;  //max
    t_motion min;  //min
    t_motion end;   //end
    t_motion tar;   //target
    t_motion sum;   //sum
    t_motion I;    //integral
}t_mouse_motion_val;    //motion value

typedef struct 
{
    float tire_diameter = 0;
    float tire_radius = 0;
    //float R;
    float Kt = 0;
    float Ke = 0;
    float truque = 0;
}t_motor;   //motor parameter

typedef struct 
{
    float Kp = 0;   //proportional gain
    float Ki = 0;   //integral gain
    float Kd = 0;   //differential gain
}t_pid; //pid parameter

typedef struct 
{
    float x_pos = 0;
    float y_pos = 0;
}t_odom;    //odometry data

typedef struct 
{
    t_pid v;    //velocity pid
    t_pid o;    //omega pid
    t_pid d;    //degree pid
    t_pid wall; //wall pid
    float Vatt = 0;
    float V_l = 0;
    float V_r = 0;
    float Duty_l = 0;
    float Duty_r = 0;
    int time_count = 0;
    t_bool flag = FALSE;
    t_motor mot;
    t_odom odom;
    t_bool log_flag = FALSE;
}t_control; //control parameter


typedef struct 
{
    unsigned char north:2;
    unsigned char east:2;
    unsigned char south:2;
    unsigned char west:2;
    t_bool flag;
}t_wall;    //wall data

typedef struct
{
    short x = 0;
    short y = 0;
    t_direction dir;
}t_pos;     //position data

typedef struct 
{
    t_pos pos;
    t_wall wall[32][32];
    unsigned char size[32][32] = {0};
    uint8_t GOAL_X = 0;
    uint8_t GOAL_Y = 0;
}t_map;     //map data

typedef struct
{
    float speed_Kp = 0;
    float speed_Ki = 0;
    float speed_Kd = 0;
    float ang_vel_Kp = 0;
    float ang_vel_Ki = 0;
    float ang_vel_Kd = 0;
    float wall_Kp = 0;
    float wall_Ki = 0;
    float wall_Kd = 0;
}t_file_pid_gain;  //parameter file

typedef struct
{
    uint16_t th_wall_fl = 0;
    uint16_t th_wall_l = 0;
    uint16_t th_wall_r = 0;
    uint16_t th_wall_fr = 0;
    uint16_t th_control_l = 0;
    uint16_t th_control_r = 0;
    uint16_t ref_l = 0;
    uint16_t ref_r = 0;
}t_file_wall_th;   //wall threshold file

typedef struct
{
    uint16_t front_l = 0;
    uint16_t front_r = 0;
    uint16_t left_fl = 0;
    uint16_t left_fr = 0;
    uint16_t right_fl = 0;
    uint16_t right_fr = 0;
    uint16_t rear_fl = 0;
    uint16_t rear_fr = 0;
}t_file_center_sens_value;


#endif // STRUCTS_HPP