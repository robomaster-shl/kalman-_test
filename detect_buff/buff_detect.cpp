#include "buff_detect.h"

/**
 * @brief 图像预处理
 * 
 * @param frame -传入原图像
 * @param enemy_color -传入敌方颜色
 */
void Max_Buff::pretreatRgb(Mat src_img, int enemy_color)
{
    //保存原图像
    this->frame = src_img;
    /* blur(frame, frame, Size(5, 5)); */
    // imshow(" ", frame);
    //转灰度图
    Mat gray_img, hsv_img;
    Mat bin_img_color, bin_img_gray;
    cvtColor(this->frame, gray_img, COLOR_BGR2GRAY);
    cvtColor(this->frame, hsv_img, COLOR_BGR2HSV_FULL);
#if IS_PARAM_ADJUSTMENT == 1
    namedWindow("src_img", WINDOW_AUTOSIZE);
    createTrackbar("H_MIN:", "src_img", &h_min, 255, NULL);
    createTrackbar("H_MAX:", "src_img", &h_max, 255, NULL);
    createTrackbar("S_MIN:", "src_img", &s_min, 255, NULL);
    createTrackbar("S_MAX:", "src_img", &s_max, 255, NULL);
    createTrackbar("V_MIN:", "src_img", &v_min, 255, NULL);
    createTrackbar("V_MAX:", "src_img", &v_max, 255, NULL);
    createTrackbar("GRAY_TH:", "src_img", &armor_gray_th, 255, NULL);
#endif
    if (enemy_color == 0)
    {
        // createTrackbar("GRAY_TH_BLUE:", "src_img", &blue_armor_gray_th, 255, NULL);
        //115 173 136 255 90 255 blue
#if IS_PARAM_ADJUSTMENT == 0
        h_min = 115;
        h_max = 173;
#endif
    }
    else if (enemy_color == 1)
    {
        //0 80 116 222 90 255 red
#if IS_PARAM_ADJUSTMENT == 0
        h_min = 0;
        h_max = 80;
#endif
    }
    inRange(hsv_img, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), bin_img_color);
    threshold(gray_img, bin_img_gray, this->armor_gray_th, 255, THRESH_BINARY);
    medianBlur(bin_img_color, bin_img_color, 3);
    /* bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(11,11)); */
    bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    // GaussianBlur(bin_img_color, bin_img_color, Size(3, 3), 0);

    Mat element = getStructuringElement(MORPH_RECT, Size(9, 9)); 
    morphologyEx(bin_img_color, bin_img_color, MORPH_CLOSE, element);

    dilate(bin_img_color, bin_img_color,
          getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));  //膨胀 晚上
    // erode(bin_img_gray, bin_img_gray,
    //     getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));  //膨胀
#if SHOW_BIN_IMG == 1    
    imshow("gray_img", bin_img_gray);
    imshow("mask", bin_img_color);
    imshow("src_img", bin_img_color);
#endif
    
    //保存处理后的图片
    this->mask = bin_img_color;
    this->gray_img = bin_img_gray;
}
/**
 * @brief 寻找大神符中心点 并保存
 * 
 * @return true 
 * @return false 
 */
bool Max_Buff::lookingCenter()
{
    findContours(this->mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    RotatedRect box;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() < 6 || contourArea(contours[i]) < 100)
        {
            continue;
        }
        box = minAreaRect(contours[i]);
        float w_h_ratio = box.size.width / box.size.height;
        float Area_ratio = contourArea(contours[i]) / (box.size.width * box.size.height);
        if (w_h_ratio < 1.1 && w_h_ratio > 0.9 && Area_ratio > 0.76 && Area_ratio < 0.86)
        {
            central_point = box.center;
            central_success = true;
            circle(src_img, central_point, 10, Scalar(0, 255, 0), -1);
        }
    }
    return central_success;
}

int Max_Buff::lookingTarget()
{
    RotatedRect box;
    int num = 0;
    input = 0;
    int min = 255;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (hierarchy[i][3] < 0 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6 || contours[i].size() < 6)
        {
            continue;
        }
        box = minAreaRect(contours[i]);
        //通过大轮廓 面积比和长宽比判断击打位置
        if ((box.size.width * box.size.height > 300) && (contourArea(contours[i]) / (box.size.width * box.size.height) > 0.8) && (((box.size.width / box.size.height) > 1.2 && (box.size.width / box.size.height) < 2.8) || ((box.size.height / box.size.width) > 1.2 && (box.size.height / box.size.width) < 2.8)))
        {
            choice_success = true;
            //调整角度范围
            if (box.angle > 90.0f)
                box.angle = box.angle - 180.0f;
            Point2f vertex[4];
            box.points(vertex);
            for (int i = 0; i < 4; i++)
            {
                line(src_img, vertex[i], vertex[(i + 1) % 4], Scalar(255, 100, 200), 5, CV_AA);
            }
            max_buff_rects.push_back(box); //保存外接矩形
            Mat src = maxbuffRoi(num);
            // imshow("s", src);
            int temp = average_Color(src);
            if (temp < min && central_success) //20红色 50蓝色
            {
                min = temp;
                hit_subscript = num;
                input++;
            }
            num++;
        }
    }
    if(!choice_success)
        return 0;
    ++find_cnt_;
    if (find_cnt_ % 2 == 0)
    { //隔帧读数据
        buff_angle_ = max_buff_rects[hit_subscript].angle;
        direction_tmp_ = getState(); //判断旋转方向 1顺时针,-1逆时针
        if (find_cnt_ == 10)
            find_cnt_ = 0;
    }
    // if (central_success)
    // {
    //     if (choice_success)
    //     {
    //         choice_success = false;
    //         central_success = false;
    //     }
    // }
    return input;
}

Mat Max_Buff::maxbuffRoi(int i)
{
    int _w = MAX(max_buff_rects[i].size.width, max_buff_rects[i].size.height);
    Point center = Point((max_buff_rects[i].center.x + central_point.x) / 2, (max_buff_rects[i].center.y + central_point.y) / 2);
    // radius = sqrt((central_point.x - max_buff_rects[i].center.x) * (central_point.x - max_buff_rects[i].center.x) + (central_point.y - max_buff_rects[i].center.y) * (central_point.y - max_buff_rects[i].center.y));
    
    //计算参数方程
    RotatedRect rect = RotatedRect(center, Size(_w * 2, _w * 2), max_buff_rects[i].angle);
    Point tl = Point(rect.center.x - (rect.size.width / 2), rect.center.y - (rect.size.height / 2));
    Point bl = Point(rect.center.x + (rect.size.width / 2), rect.center.y + (rect.size.height / 2));
    if (tl.x < 0)
    {
        tl.x = 0;
    }
    if (bl.x > CAMERA_RESOLUTION_COLS)
    {
        bl.x = CAMERA_RESOLUTION_COLS;
    }
    if (tl.y < 0)
    {
        tl.y = 0;
    }
    if (bl.y > CAMERA_RESOLUTION_ROWS)
    {
        bl.y = CAMERA_RESOLUTION_ROWS;
    }
    Rect rects = Rect(tl, bl);
    // rectangle(src_img, rects, Scalar(0, 0, 255), 5, 8);
    Mat src = mask(rects);
    return src;
}

/**
 * @brief 计算图像中像素点的平均强度
 * 
 * @param roi 传入需要计算的图像
 * @return int 返回平均强度
 */
int Max_Buff::average_Color(Mat roi)
{
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    cout << average_intensity << endl;
    return average_intensity;
}

/**
 * @brief 计算其他击打位置坐标（参数方程计算有问题）
 * 
 */
void Max_Buff::calculatingCoordinates(int i)
{
    radius = sqrt((central_point.x - max_buff_rects[i].center.x) * (central_point.x - max_buff_rects[i].center.x) + (central_point.y - max_buff_rects[i].center.y) * (central_point.y - max_buff_rects[i].center.y));
    
    small_radius = (sqrt(pow(max_buff_rects[i].size.width, 2) + pow(max_buff_rects[i].size.height, 2))) / 2;
    circle(src_img, central_point, radius, Scalar(255, 255, 0), 3);
    circle(src_img, max_buff_rects[i].center, 7, Scalar(0, 0, 255), -1);
    line(src_img, central_point, max_buff_rects[i].center, Scalar(0, 0, 255), 3);
    //圆一
    float a1 = central_point.x;
    float b1 = central_point.y;
    float R1 = radius;
    //圆二
    float a2 = max_buff_rects[i].center.x;
    float b2 = max_buff_rects[i].center.y;
    float R2;
    if (true) //小能量机关补偿
    {
        R2 = sqrt(2 * MAX_BUFF_RADIUS * MAX_BUFF_RADIUS * (1 - cos(CV_PI * MAX_BUFF_RADIUS * make_up_angle / 180))) / 10;
    }
    else
    {
        // double spt_t = (timing_point_1 - timing_point_2) / getTickFrequency(); //现在的单位为秒
        // timing_point_2 = getTickCount();
        // make_up_angle = 0.785 * sin(spt_t * 1.884) + 1.305;
        // cout << spt_t << endl;
        R2 = sqrt(2 * MAX_BUFF_RADIUS * MAX_BUFF_RADIUS * (1 - cos(CV_PI * MAX_BUFF_RADIUS * make_up_angle / 180))) / 10;
    }

    float a1_square = a1 * a1;
    float b1_square = b1 * b1;
    float R1_square = R1 * R1;
    float a2_square = a2 * a2;
    float b2_square = b2 * b2;
    float R2_square = R2 * R2;
    float subs1 = a1_square - 2 * a1 * a2 + a2_square + b1_square - 2 * b1 * b2 + b2_square;
    float subs2 = -R1_square * a1 + R1_square * a2 + R2_square * a1 - R2_square * a2 + a1_square * a1 - a1_square * a2 - a1 * a2_square + a1 * b1_square - 2 * a1 * b1 * b2 + a1 * b2_square + a2_square * a2 + a2 * b1_square - 2 * a2 * b1 * b2 + a2 * b2_square;
    float subs3 = -R1_square * b1 + R1_square * b2 + R2_square * b1 - R2_square * b2 + a1_square * b1 + a1_square * b2 - 2 * a1 * a2 * b1 - 2 * a1 * a2 * b2 + a2_square * b1 + a2_square * b2 + b1_square * b1 - b1_square * b2 - b1 * b2_square + b2_square * b2;
    float sigma = sqrt((R1_square + 2 * R1 * R2 + R2_square - a1_square + 2 * a1 * a2 - a2_square - b1_square + 2 * b1 * b2 - b2_square) * (-R1_square + 2 * R1 * R2 - R2_square + subs1));

    if (abs(subs1) > 0.0000001) //分母不为0
    {
        calculation_position[0].x = (subs2 - sigma * b1 + sigma * b2) / (2 * subs1);
        calculation_position[1].x = (subs2 + sigma * b1 - sigma * b2) / (2 * subs1);

        calculation_position[0].y = (subs3 + sigma * a1 - sigma * a2) / (2 * subs1);
        calculation_position[1].y = (subs3 - sigma * a1 + sigma * a2) / (2 * subs1);
    }
    if (direction_tmp_ > 0) //顺时针
    {

        pre_center = calculation_position[1];
    }
    else if (direction_tmp_ < 0)
    {
        pre_center = calculation_position[0];
    }
    else
    {
        pre_center = max_buff_rects[i].center;
    }
    circle(src_img, pre_center, 10, Scalar(0, 0, 255), -1);
}

int Max_Buff::getState()
{
    diff_angle_ = buff_angle_ - last_angle;
    last_angle = buff_angle_;
    if (fabs(diff_angle_) < 10 && fabs(diff_angle_) > 1e-6)
    {
        d_angle_ = (1 - REVISE) * d_angle_ + REVISE * diff_angle_;
    }
    if (d_angle_ > 1.5)
        return 1;
    else if (d_angle_ < -1.5)
        return -1;
    else
        return 0;
}