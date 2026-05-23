/**
 * @file vision.cpp
 * @brief 红色箱子视觉识别与位姿估计节点
 * 
 * 功能说明：
 * 1. 从 USB 摄像头读取图像
 * 2. 通过 HSV 颜色分割检测红色箱子
 * 3. 使用卡尔曼滤波器平滑角点位置
 * 4. 使用 solvePnP 计算相机到物体的位姿
 * 5. 发布 TF 变换（camera_link -> target_object）
 * 6. 可视化显示检测结果
 */

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace cv;

// ================================================================
//  常量配置区 —— 只改这里就能适配你的箱子
// ================================================================

/**
 * 箱子物理参数
 */
static const float BOX_MM      = 350.0f;      // 箱子顶面边长，单位：mm
static const float HALF        = BOX_MM / 2.f;
static const double MIN_AREA   = 8000.0;      // 最小有效红色面积阈值（像素），用于过滤噪声

/**
 * 平滑参数
 */
static const int   SMOOTH_N    = 6;           // 位姿平滑的滑动窗口帧数

/**
 * TF 坐标系名称
 */
static const char* CAMERA_FRAME_ID = "camera_link";     // 相机坐标系
static const char* OBJECT_FRAME_ID = "target_object";   // 目标物体坐标系

/**
 * 物体坐标系定义：顶面中心为原点，单位 mm
 * 四个角点的 3D 坐标（用于 PnP 求解）
 * 顺序：[左上, 右上, 右下, 左下]
 */
static const vector<Point3f> OBJ_PTS = {
    {-HALF, -HALF, 0},  // 0 左上
    { HALF, -HALF, 0},  // 1 右上
    { HALF,  HALF, 0},  // 2 右下
    {-HALF,  HALF, 0}   // 3 左下
};



// ===== USB摄像头标定参数（你的参数）=====
/**
 * 相机内参矩阵 K
 * 格式：[fx,  0, cx]
 *      [ 0, fy, cy]
 *      [ 0,  0,  1]
 */
static const Mat K = (Mat_<double>(3,3) <<
    786.19375828781722, 0.,                  668.98017421012958,
    0.,                 791.8946129798486,   373.97215705020159,
    0.,                 0.,                  1.);

/**
 * 相机畸变系数 D
 * 格式：[k1, k2, p1, p2, k3]
 * k1, k2, k3: 径向畸变系数
 * p1, p2: 切向畸变系数
 */
static const Mat D = (Mat_<double>(1,5) <<
     0.06420428268786578,
     0.083133304993222509,
     0.0019411199567928366,
    -0.0032091194694991478,
    -0.34311667376016997);























// ================================================================
//  卡尔曼滤波器封装（对每个角点单独建一个 4 状态 KF：[x, y, vx, vy]）
// ================================================================
/**
 * @struct CornerKF
 * @brief 单个角点的卡尔曼滤波器封装
 * 
 * 状态向量：[x, y, vx, vy]（位置 + 速度）
 * 测量向量：[x, y]（仅位置）
 * 
 * 作用：平滑角点位置，减少抖动，在检测失败时进行预测
 */
struct CornerKF
{
    KalmanFilter kf;            // OpenCV 卡尔曼滤波器对象
    bool initialized = false;  // 是否已初始化
    
    /**
     * @brief 初始化卡尔曼滤波器
     * @param pt 初始角点位置
     */
    void init(Point2f pt)
    {
        // 初始化卡尔曼滤波器：4个状态变量，2个测量变量
        kf.init(4, 2, 0, CV_64F);
        
        // 状态转移矩阵 A（匀速运动模型）
        // [1, 0, 1, 0]   x' = x + vx
        // [0, 1, 0, 1]   y' = y + vy
        // [0, 0, 1, 0]   vx' = vx
        // [0, 0, 0, 1]   vy' = vy
        setIdentity(kf.transitionMatrix);
        kf.transitionMatrix.at<double>(0,2) = 1.0;
        kf.transitionMatrix.at<double>(1,3) = 1.0;
        
        // 测量矩阵 H（只测量位置）
        // [1, 0, 0, 0]   z_x = x
        // [0, 1, 0, 0]   z_y = y
        kf.measurementMatrix = Mat::zeros(2, 4, CV_64F);
        kf.measurementMatrix.at<double>(0,0) = 1.0;
        kf.measurementMatrix.at<double>(1,1) = 1.0;
        
        // 过程噪声协方差矩阵 Q（模型不确定性）
        // 值越小，滤波器越信任模型预测
        setIdentity(kf.processNoiseCov, Scalar(0.05));
        
        // 测量噪声协方差矩阵 R（测量不确定性）
        // 值越大，滤波器越平滑，但滞后越明显
        setIdentity(kf.measurementNoiseCov, Scalar(0.8));
        
        // 初始后验误差协方差矩阵
        setIdentity(kf.errorCovPost, Scalar(1));
        
        // 初始状态：位置为测量值，速度为 0
        kf.statePost = (Mat_<double>(4,1) << pt.x, pt.y, 0, 0);
        initialized = true;
    }
    
    /**
     * @brief 更新卡尔曼滤波器（预测 + 校正）
     * @param measured 测量到的角点位置
     * @return 滤波后的角点位置
     */
    Point2f update(Point2f measured)
    {
        if (!initialized) init(measured);
        
        // 预测步骤：根据状态转移矩阵预测下一状态
        Mat pred = kf.predict();
        
        // 测量步骤：构造测量向量
        Mat meas = (Mat_<double>(2,1) << measured.x, measured.y);
        
        // 校正步骤：融合预测和测量
        Mat est = kf.correct(meas);
        
        return Point2f((float)est.at<double>(0), (float)est.at<double>(1));
    }
    
    /**
     * @brief 仅进行预测（检测失败时使用）
     * @return 预测的角点位置
     */
    Point2f predictOnly()
    {
        Mat pred = kf.predict();
        return Point2f((float)pred.at<double>(0), (float)pred.at<double>(1));
    }
};
























// ================================================================
//  工具函数
// ================================================================
/**
 * @brief 对四个角点进行排序
 * @param in 输入的四个角点（无序）
 * @return 排序后的角点：[左上, 右上, 右下, 左下]
 * 
 * 算法原理：
 * - s = x + y，最小的是左上角，最大的是右下角
 * - d = x - y，最大的是右上角，最小的是左下角
 */
vector<Point2f> sortCorners(const vector<Point2f>& in)
{
    vector<Point2f> r(4);
    vector<float> s(4), d(4);
    
    // 计算每个点的 s 和 d 值
    for (int i = 0; i < 4; i++) { 
        s[i] = in[i].x + in[i].y; 
        d[i] = in[i].x - in[i].y; 
    }
    
    // 找出四个角点
    int tl=0, br=0, tr=0, bl=0;
    for (int i = 1; i < 4; i++)
    {
        if (s[i] < s[tl]) tl = i;   // s 最小 -> 左上
        if (s[i] > s[br]) br = i;   // s 最大 -> 右下
        if (d[i] > d[tr]) tr = i;   // d 最大 -> 右上
        if (d[i] < d[bl]) bl = i;   // d 最小 -> 左下
    }
    
    r[0]=in[tl]; r[1]=in[tr]; r[2]=in[br]; r[3]=in[bl];
    return r;
}








/**
 * @brief 旋转矩阵转欧拉角（ZYX 顺序）
 * @param R 旋转矩阵 (3x3)
 * @return 欧拉角，单位：度
 *         [Roll, Pitch, Yaw] = [绕X轴旋转, 绕Y轴旋转, 绕Z轴旋转]
 */
Vec3d euler(const Mat& R)
{
    // 计算中间变量 sy = sqrt(r11^2 + r21^2)
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) +
                     R.at<double>(1,0)*R.at<double>(1,0));
    
    // 判断是否奇异点（万向锁）
    bool sg = sy < 1e-6;
    
    // 计算欧拉角
    double x = sg ? atan2(-R.at<double>(1,2), R.at<double>(1,1))
                  : atan2( R.at<double>(2,1), R.at<double>(2,2));
    double y = atan2(-R.at<double>(2,0), sy);
    double z = sg ? 0 : atan2(R.at<double>(1,0), R.at<double>(0,0));
    
    return Vec3d(x * 180/CV_PI, y * 180/CV_PI, z * 180/CV_PI);
}











/**
 * @brief 在图像上绘制带描边的文字（提高可读性）
 * @param img 目标图像
 * @param text 文字内容
 * @param org 文字左下角位置
 * @param scale 字体大小
 * @param color 文字颜色
 * @param thick 文字粗细
 */
void putLabel(Mat& img, const string& text, Point org,
              double scale=0.65, Scalar color=Scalar(0,255,255), int thick=2)
{
    // 先绘制黑色描边
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0,0,0), thick+2);
    // 再绘制彩色文字
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}










// ================================================================
//  核心：从彩色帧里提取箱子四角
//  返回 false 表示本帧检测失败
// ================================================================
/**
 * @brief 检测红色箱子的四个角点
 * @param frame 输入图像（BGR）
 * @param prevGray 上一帧灰度图（预留，当前未使用）
 * @param corners 输出的四个角点（排序后）
 * @param debugMask 调试用的红色区域掩码
 * @return 检测是否成功
 * 
 * 检测流程：
 * 1. HSV 颜色分割提取红色区域
 * 2. 形态学处理（闭运算填洞，开运算去噪）
 * 3. 找最大轮廓
 * 4. 凸包 → 多边形逼近 → 提取四角点
 * 5. 亚像素精化
 */
bool detectBoxCorners(const Mat& frame, const Mat& prevGray,
                      vector<Point2f>& corners, Mat& debugMask)
{
    // ---- 1. HSV 红色分割 ----
    // 转换到 HSV 颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 红色在 HSV 中分布在两端（0° 附近和 180° 附近）
    // 需要两个阈值范围，然后合并
    Mat m1, m2, mask;
    inRange(hsv, Scalar(0,  80, 60), Scalar(10,  255, 255), m1);      // 低红色 (0-10°)
    inRange(hsv, Scalar(165, 80, 60), Scalar(180, 255, 255), m2);     // 高红色 (165-180°)
    mask = m1 | m2;  // 合并两个区域
    
    // ---- 2. 形态学：先闭运算填洞，再开运算去噪 ----
    Mat k5 = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat k3 = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_CLOSE, k5, Point(-1,-1), 2);  // 闭运算：填充内部空洞
    morphologyEx(mask, mask, MORPH_OPEN,  k3, Point(-1,-1), 1);  // 开运算：去除外部噪点
    
    debugMask = mask.clone();
    
    // ---- 3. 找轮廓，取最大面积 ----
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // 没有找到轮廓
    if (contours.empty()) return false;
    
    // 找出面积最大的轮廓（假设最大的是箱子）
    int best = 0;
    double maxA = 0;
    for (int i = 0; i < (int)contours.size(); i++)
    {
        double a = contourArea(contours[i]);
        if (a > maxA) { maxA = a; best = i; }
    }
    
    // 面积太小，可能是噪声
    if (maxA < MIN_AREA) return false;
    
    // ---- 4. 凸包 → 多边形逼近 → 强制4点 ----
    // 先计算凸包，确保轮廓是凸的
    vector<Point> hull;
    convexHull(contours[best], hull);
    
    // 多边形逼近：尝试不同精度，直到逼近结果 <= 5 点
    vector<Point> poly;
    double peri = arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.15; eps += 0.01)
    {
        approxPolyDP(hull, poly, eps * peri, true);
        if ((int)poly.size() <= 5) break;
    }
    
    // 用最小外接旋转矩形兜底，保证一定能得到 4 个点
    RotatedRect rr = minAreaRect(hull);
    Point2f rpts[4];
    rr.points(rpts);
    
    // 如果逼近结果恰好是 4 点且是凸多边形，使用逼近结果（更精确）
    // 否则使用旋转矩形的 4 个角（更稳定）
    vector<Point2f> raw(4);
    if ((int)poly.size() == 4 && isContourConvex(poly))
    {
        for (int i = 0; i < 4; i++) raw[i] = Point2f((float)poly[i].x, (float)poly[i].y);
    }
    else
    {
        for (int i = 0; i < 4; i++) raw[i] = rpts[i];
    }
    
    // ---- 5. 排序 [左上 右上 右下 左下] ----
    raw = sortCorners(raw);
    
    // ---- 6. 亚像素精化 ----
    // 转换为灰度图
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    
    // 确保角点在图像范围内
    for (auto& p : raw)
    {
        p.x = clamp(p.x, 0.f, (float)(frame.cols - 1));
        p.y = clamp(p.y, (float)0, (float)(frame.rows - 1));
    }
    
    // 亚像素角点精化（提高精度到亚像素级）
    TermCriteria tc(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01);
    cornerSubPix(gray, raw, Size(7, 7), Size(-1, -1), tc);
    
    corners = raw;
    return true;
}

// ================================================================
//  距离 / 位姿平滑（对 tvec 做滑动均值）
// ================================================================
/**
 * @struct PoseSmootherVec3
 * @brief 对位姿向量进行滑动平均平滑
 * 
 * 作用：减少位姿抖动，使输出更稳定
 */
struct PoseSmootherVec3
{
    deque<Vec3d> buf;   // 历史数据缓冲区
    int maxN;           // 最大帧数
    
    explicit PoseSmootherVec3(int n) : maxN(n) {}
    
    /**
     * @brief 添加新数据并返回平滑结果
     * @param v 新的位姿向量
     * @return 平滑后的位姿向量
     */
    Vec3d push(Vec3d v)
    {
        buf.push_back(v);
        if ((int)buf.size() > maxN) buf.pop_front();  // 保持窗口大小
        
        // 计算均值
        Vec3d sum(0,0,0);
        for (auto& x : buf) sum += x;
        return sum * (1.0 / buf.size());
    }
};

// ================================================================
//  main
// ================================================================
/**
 * @brief 主函数
 * 
 * 流程：
 * 1. 初始化 ROS2 节点和 TF 广播器
 * 2. 打开 USB 摄像头
 * 3. 计算去畸变映射
 * 4. 循环处理每一帧：
 *    a. 去畸变
 *    b. 检测箱子角点
 *    c. 卡尔曼滤波平滑角点
 *    d. solvePnP 计算位姿
 *    e. 发布 TF 变换
 *    f. 可视化显示
 */
int main(int argc, char** argv)
{
    // ========== 1. ROS2 初始化 ==========
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vision_node");
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // ---------- 启动 USB 摄像头 ----------
    VideoCapture cap;
    const int camera_index = 4;  // 摄像头设备索引
    
    // 尝试多种后端打开摄像头（提高兼容性）
    const std::array<int, 3> backends = {CAP_V4L2, CAP_ANY, CAP_GSTREAMER};
    bool opened = false;
    for (int backend : backends) {
        cap.release();
        if (!cap.open(camera_index, backend)) {
            cerr << "后端 " << backend << " 打开摄像头索引 " << camera_index
                 << " 失败\n";
            continue;
        }

        // 获取并显示后端名称
        std::string backend_name = "unknown";
        try {
            backend_name = cap.getBackendName();
        } catch (...) {
            backend_name = "unavailable";
        }
        cout << "摄像头打开成功，索引=" << camera_index
             << "，后端=" << backend_name << "\n";
        opened = true;
        break;
    }

    if (!opened || !cap.isOpened()) {
        cerr << "无法打开摄像头，请检查设备索引和占用情况\n";
        rclcpp::shutdown();
        return -1;
    }
    
    // 设置摄像头参数
    cap.set(CAP_PROP_FRAME_WIDTH,  1280);   // 分辨率宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);    // 分辨率高度
    cap.set(CAP_PROP_FPS, 60);              // 帧率
    
    // 显示相机参数
    cout << "K:\n" << K << "\nD:\n" << D << "\n";
    
    // ========== 2. 计算去畸变映射 ==========
    Mat frame_tmp;
    cap >> frame_tmp;
    if (frame_tmp.empty()) {
        cerr << "摄像头已打开，但读取首帧失败\n";
        cap.release();
        rclcpp::shutdown();
        return -1;
    }
    Size imgSize = frame_tmp.size();
    
    // 计算去畸变后的新相机矩阵
    Mat newK = getOptimalNewCameraMatrix(K, D, imgSize, 0.0, imgSize);
    
    // 预计算去畸变映射表（提高实时性能）
    Mat mapX, mapY;
    initUndistortRectifyMap(K, D, Mat(), newK, imgSize, CV_32FC1, mapX, mapY);
    
    // ========== 3. 初始化滤波器 ==========
    // 卡尔曼滤波器：4 个角点各一个
    array<CornerKF, 4> kfs;
    
    // 位姿平滑器
    PoseSmootherVec3 tvecSmoother(SMOOTH_N);
    
    // 上一帧灰度图（预留，当前未使用）
    Mat prevGray;
    bool hasPrev = false;
    
    // 零畸变系数（用于去畸变后的图像）
    Mat zeroDist = Mat::zeros(1, 5, CV_64F);
    
    // ========== 4. 主循环 ==========
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // ----- 取帧 -----
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        
        // 去畸变（使用预计算的映射表，速度快）
        Mat undistorted;
        remap(frame, undistorted, mapX, mapY, INTER_LINEAR);
        
        // 准备显示图像和调试数据
        Mat show = undistorted.clone();
        Mat debugMask;
        vector<Point2f> rawCorners;
        
        // ----- 检测箱子角点 -----
        bool detected = detectBoxCorners(undistorted, prevGray, rawCorners, debugMask);
        
        // ----- 卡尔曼平滑角点 -----
        vector<Point2f> smoothCorners(4);
        if (detected)
        {
            // 检测成功：使用测量值更新卡尔曼滤波器
            for (int i = 0; i < 4; i++)
                smoothCorners[i] = kfs[i].update(rawCorners[i]);
        }
        else
        {
            // 检测失败：使用卡尔曼滤波器预测
            for (int i = 0; i < 4; i++)
            {
                if (kfs[i].initialized)
                    smoothCorners[i] = kfs[i].predictOnly();
                else
                    smoothCorners[i] = Point2f(0,0);
            }
        }
        
        bool anyInited = kfs[0].initialized;
        
        // ----- 画角点框 -----
        if (anyInited)
        {
            // 绘制箱子边框和角点
            for (int i = 0; i < 4; i++)
            {
                Point2f a = smoothCorners[i];
                Point2f b = smoothCorners[(i+1)%4];
                
                // 绿色边框
                line(show, a, b, Scalar(0, 220, 0), 3, LINE_AA);
                
                // 角点圆（红色）
                circle(show, a, 7, Scalar(0, 0, 255), -1, LINE_AA);
                
                // 角点编号
                putLabel(show, to_string(i), a + Point2f(8, -8),
                        0.65, Scalar(255, 255, 0));
            }
            
            // ----- solvePnP 计算位姿 -----
            Mat rvec, tvec;
            bool pnp_ok = solvePnP(OBJ_PTS, smoothCorners, newK, zeroDist,
                                   rvec, tvec, false, SOLVEPNP_IPPE);
            
            if (pnp_ok)
            {
                // ---- tvec 平滑 ----
                Vec3d tv(tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2));
                Vec3d tvSmooth = tvecSmoother.push(tv);
                
                double X    = tvSmooth[0];
                double Y    = tvSmooth[1];
                double Z    = tvSmooth[2];
                double dist = sqrt(X*X + Y*Y + Z*Z);  // 计算距离
                
                // ---- 旋转矩阵 / 欧拉角 ----
                Mat R;
                Rodrigues(rvec, R);  // 旋转向量转旋转矩阵
                Vec3d eu = euler(R);  // 旋转矩阵转欧拉角

                // ---- 发布 TF 变换 ----
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = node->get_clock()->now();
                transformStamped.header.frame_id = CAMERA_FRAME_ID;
                transformStamped.child_frame_id = OBJECT_FRAME_ID;
                
                // 平移部分（mm -> m）
                transformStamped.transform.translation.x = X / 1000.0;
                transformStamped.transform.translation.y = Y / 1000.0;
                transformStamped.transform.translation.z = Z / 1000.0;

                // 旋转部分（旋转矩阵 -> 四元数）
                tf2::Matrix3x3 tf_rot(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q;
                tf_rot.getRotation(q);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                
                tf_broadcaster->sendTransform(transformStamped);
                
                // ---- 投影坐标轴（可视化物体坐标系） ----
                Mat tvecSmoothed = (Mat_<double>(3,1) << X, Y, Z);
                
                // 物体坐标系的三个坐标轴端点
                vector<Point3f> axisPts = {
                    {0,    0,    0},   // 原点
                    {100,  0,    0},   // X 轴端点（红色）
                    {0,    100,  0},   // Y 轴端点（绿色）
                    {0,    0,    100}  // Z 轴端点（蓝色，朝向相机方向）
                };
                vector<Point2f> axisImg;
                projectPoints(axisPts, rvec, tvecSmoothed, newK, zeroDist, axisImg);
                
                // 画坐标轴箭头
                arrowedLine(show, axisImg[0], axisImg[1], Scalar(0,   0,   255), 3, LINE_AA, 0, 0.2);  // X-红
                arrowedLine(show, axisImg[0], axisImg[2], Scalar(0,   255, 0  ), 3, LINE_AA, 0, 0.2);  // Y-绿
                arrowedLine(show, axisImg[0], axisImg[3], Scalar(255, 0,   0  ), 3, LINE_AA, 0, 0.2);  // Z-蓝
                
                // 坐标轴标签
                putLabel(show, "X", axisImg[1] + Point2f(5, 0),  0.6, Scalar(0,   0,   255));
                putLabel(show, "Y", axisImg[2] + Point2f(5, 0),  0.6, Scalar(0,   255, 0  ));
                putLabel(show, "Z", axisImg[3] + Point2f(5, 0),  0.6, Scalar(255, 100, 0  ));
                
                // ---- 投影箱子中心点 ----
                vector<Point2f> centerImg;
                projectPoints(vector<Point3f>{{0,0,0}},
                             rvec, tvecSmoothed, newK, zeroDist, centerImg);
                if (!centerImg.empty())
                {
                    // 十字准星标记中心点
                    int cx = (int)centerImg[0].x;
                    int cy = (int)centerImg[0].y;
                    line(show, Point(cx-14, cy),   Point(cx+14, cy),   Scalar(0,255,255), 2, LINE_AA);
                    line(show, Point(cx, cy-14),   Point(cx, cy+14),   Scalar(0,255,255), 2, LINE_AA);
                    circle(show, centerImg[0], 5, Scalar(0,255,255), -1, LINE_AA);
                }
                
                // ---- 信息面板（左上角半透明背景） ----
                {
                    // 画半透明黑底
                    Mat overlay = show.clone();
                    rectangle(overlay, Point(10, 10), Point(500, 185), Scalar(0,0,0), FILLED);
                    addWeighted(overlay, 0.45, show, 0.55, 0, show);
                    
                    int bx = 20, by = 35;
                    int dy = 30;
                    char buf[256];
                    
                    // 距离（最重要，大字）
                    sprintf(buf, "Distance : %.1f mm", dist);
                    putLabel(show, buf, Point(bx, by),
                            0.78, Scalar(0, 255, 100), 2);
                    
                    // XYZ 坐标
                    sprintf(buf, "X=%.1f  Y=%.1f  Z=%.1f  (mm)", X, Y, Z);
                    putLabel(show, buf, Point(bx, by + dy),
                            0.62, Scalar(0, 220, 255));
                    
                    // 欧拉角
                    sprintf(buf, "Roll=%.1f  Pitch=%.1f  Yaw=%.1f  (deg)",
                           eu[0], eu[1], eu[2]);
                    putLabel(show, buf, Point(bx, by + dy*2),
                            0.62, Scalar(255, 200, 0));
                    
                    // 检测状态
                    string status = detected ? "[ DETECT: OK ]" : "[ DETECT: LOST - KF predict ]";
                    Scalar  scol  = detected ? Scalar(0,255,0) : Scalar(0,100,255);
                    putLabel(show, status, Point(bx, by + dy*3),
                            0.62, scol);
                    
                    // 角点像素坐标
                    sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                           smoothCorners[0].x, smoothCorners[0].y,
                           smoothCorners[1].x, smoothCorners[1].y,
                           smoothCorners[2].x, smoothCorners[2].y,
                           smoothCorners[3].x, smoothCorners[3].y);
                    putLabel(show, buf, Point(bx, by + dy*4),
                            0.52, Scalar(180, 180, 180));
                    
                    // 控制台输出
                    printf("\rDist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  "
                          "RPY=[%.1f, %.1f, %.1f]deg   ",
                          dist, X, Y, Z, eu[0], eu[1], eu[2]);
                    fflush(stdout);
                }
            }
        }
        
        // ---- 更新 prevGray ----
        cvtColor(undistorted, prevGray, COLOR_BGR2GRAY);
        hasPrev = true;
        
        // ---- 显示 ----
        // 把 mask 缩小显示在右下角，不遮挡主画面
        {
            Mat maskBGR, maskSmall;
            cvtColor(debugMask, maskBGR, COLOR_GRAY2BGR);
            resize(maskBGR, maskSmall, Size(320, 180));
            int ox = show.cols - 325;
            int oy = show.rows - 185;
            maskSmall.copyTo(show(Rect(ox, oy, maskSmall.cols, maskSmall.rows)));
            putLabel(show, "RedMask", Point(ox+5, oy+18), 0.55, Scalar(200,200,200));
        }
        
        imshow("Box PnP", show);
        
        // 按键处理
        char key = (char)waitKey(1);
        if (key == 27) break;  // ESC 退出
        
        // 按 r：重置卡尔曼滤波器
        if (key == 'r' || key == 'R')
        {
            for (auto& kf : kfs) kf.initialized = false;
            tvecSmoother.buf.clear();
            cout << "\n[INFO] Kalman reset.\n";
        }
    }
    
    // ========== 5. 清理资源 ==========
    cap.release();
    destroyAllWindows();
    rclcpp::shutdown();
    cout << "\n[INFO] Exit.\n";
    return 0;
}
