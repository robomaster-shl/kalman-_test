#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"
#include "kalmantest/kalmantest.h"
#include "detect_buff/buff_detect.h"
#include "stdlib.h"

class WorKing
{
public:
    WorKing();
    ~WorKing();
    void Run();              //主函数
    void ddd();              //拍摄图片
    void Run_MAX_Talisman(); //大神符
    
    void Return_zero();//归零
    void Angle_compensate();//pnp计算角度补偿
    void Mode_Selection(); // 模式选择
    void Automatic_fire();//自动开火
    bool top();//小陀螺判断(陀螺仪)
    ImageProcess img;
    Max_Buff buff;
    SerialPort serial;
    SolveP4p pnp;
    VideoCapture capture;
    VideoCap cap;
    RM_kalmanfilter kalman;
    
    
    Mat frame;//保留原图
    // Mat src_img;//ROI 图像
    
    int enemy_color = 1;//敌方颜色
    int pattern = 0;//模式选择
    int firing = 0;//射速15 18 30
    int fire_num = 0;//开火计数
    bool data_success = false;
    bool judge_top = false;//小陀螺或者扭腰判断
    int gyro_arr[50] = {0};//陀螺仪数据储存 50帧分析一次
    int _yaw = 0;
    float yaw = 0;
    int _pitch = 0;
    float pitch = 0;
    int depth = 0;
    int data_type = 0;
    int is_shooting = 0;

    int offset_x = 100;
    int offset_y = 250;
    int _offset_x = 1;
    int _offset_y = 1;


class Kalman1
{
public:
    Kalman1(){
        Q_ = 0.001f;
        R_ = 0.005f;
        t_ = 0.005f;
        x_ = 0.0f;
        p_ = 0.5f;
    }
    Kalman1(float Q, float R, float t, float x0, float p0)
    {
        Q_ = Q;
        R_ = R;
        t_ = t;
        x_ = x0;
        p_ = p0;
    }
    void setParam(int R, int Q, int t){
        if(R<1)
            R=1;
        if(Q<1)
            Q=1;
        if(t<1)
            t=1;
        R_ = static_cast<float>(R*0.001f);
        Q_ = static_cast<float>(Q*0.001f);
        t_ = static_cast<float>(t);
    }
    float run(float data){

        x_pre_ = x_;                                      //x(k|k-1) = AX(k-1|k-1)+BU(k)
        p_pre_ = p_ + Q_;                              //p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);               //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data - x_pre_);          //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                   //p(k|k) = (I-kg(k)H)P(k|k-1)
        return x_;
    }
    float merge_run(float data1, float data2)
    {
        x_pre_ = data1;
        p_pre_ = p_ + Q_;                              //p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);               //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data2 - x_pre_);          //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                   //p(k|k) = (I-kg(k)H)P(k|k-1)
        return x_;
    }
public:
    float R_;
    float Q_;
    float p_pre_;
    float x_pre_;
    float x_;
    float p_;
    float kg_;
    float t_;

};
};




