#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <cmath>

#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
}

// ============================================================
//                      可配置参数
// ============================================================

// ---- USB 相机 ----
constexpr int   CAMERA_DEVICE_ID = 0;     // /dev/videoX
constexpr int   FRAME_WIDTH      = 640;
constexpr int   FRAME_HEIGHT     = 480;

// ---- 相机内参（标定结果）----
// 标定@640x480（camera_matrix）
constexpr double FX = 325.53172475155594;
constexpr double FY = 325.5127347957183;
constexpr double CX = 323.4747330012186;
constexpr double CY = 251.68298556701887;

// plumb_bob 畸变 [k1, k2, p1, p2, k3]
constexpr double K1 = -0.0011399200886537332;
constexpr double K2 = -0.09491210221445738;
constexpr double P1 = -0.0003002068636793419;
constexpr double P2 =  0.00322949710749793;
constexpr double K3 =  0.09408591579045257;

// ---- 窗口 ----
constexpr int   WINDOW_WIDTH  = 1280;
constexpr int   WINDOW_HEIGHT = 960;

// ---- AprilTag 检测器 ----
constexpr float QUAD_DECIMATE = 1.0f;   // 1.0=全分辨率；想更快设 2.0（320x240等效检测）
constexpr float QUAD_SIGMA    = 0.0f;
constexpr int   NTHREADS      = 4;
constexpr int   REFINE_EDGES  = 0;      // 关闭边缘细化，大幅提速

// ---- 帧平均 ----
constexpr int   FRAME_AVG_COUNT = 1;      // 设为 1 = 每帧检测，零延迟

// ---- Tag 物理尺寸（米）----
constexpr float TAG_SIZE = 0.08f;

// ---- 坐标轴长度（米）----
constexpr float AXIS_LENGTH = 0.02f;

// ---- ROS TF ----
constexpr char  PARENT_FRAME[]      = "usb_camera";   // 父坐标系（相机）
constexpr char  MERGED_FRAME[]      = "R1_base_footprint";   // 融合后的坐标系（车体）
constexpr int   TAG_ID_A = 0;                         // 要融合的 tag A
constexpr int   TAG_ID_B = 1;                         // 要融合的 tag B
// 安装偏移：tag 中心 → 车体原点，在【车体坐标系】下定义（米）
constexpr double OFFSET_X = 0.0;                      // 车体 X 偏移
constexpr double OFFSET_Y = 0.0;                      // 车体 Y 偏移
constexpr double OFFSET_Z = 0.0;                      // 车体 Z 偏移

// ---- 平滑 / 丢帧保持 ----
constexpr double SMOOTH_ALPHA = 0.3;                  // EMA/SLERP 系数，越小越平滑(0~1)
constexpr int    HOLD_FRAMES  = 10;                   // 检测丢失后最多保持发布上一次位姿的帧数
constexpr int    PRINT_EVERY_N = 5;                   // 每隔 N 帧打印一次控制台输出（减少 I/O）

// ============================================================
//                 线程共享数据结构
// ============================================================

struct TagPose
{
    int      id;
    double   t[3];       // 平移 X, Y, Z
    double   R[9];       // 旋转矩阵 3×3 (row-major)
};

// 融合后的均值（主线程写入，ROS 线程读取）
struct MergedPose
{
    bool    valid = false;
    double  x_avg = 0.0;    // 发布的车体原点 X（相机系，含偏移）
    double  z_avg = 0.0;    // 发布的车体原点 Z（相机系，含偏移）
    double  t[3];           // 完整平移（相机系下的车体原点）
    double  R[9];           // 完整旋转矩阵（R_cam_car，车体系→相机系）
};

// ============================================================
//                     ROS2 TF 发布线程
// ============================================================

static std::mutex              g_mutex;
static std::condition_variable g_cv;
static MergedPose              g_merged;
static bool                    g_new_data = false;
static std::atomic<bool>       g_running{true};

void ros_tf_thread()
{
    auto node = std::make_shared<rclcpp::Node>("apriltag_tf_publisher");
    tf2_ros::TransformBroadcaster broadcaster(node);
    rclcpp::Rate rate(30);  // 30Hz

    while (rclcpp::ok() && g_running.load())
    {
        MergedPose local;

        {
            std::unique_lock<std::mutex> lock(g_mutex);
            g_cv.wait_for(lock, std::chrono::milliseconds(33),
                          []{ return g_new_data; });
            if (g_new_data)
            {
                local      = g_merged;
                g_new_data = false;
            }
        }

        if (local.valid)
        {
            geometry_msgs::msg::TransformStamped ts;

            ts.header.stamp    = node->get_clock()->now();
            ts.header.frame_id = PARENT_FRAME;
            ts.child_frame_id  = MERGED_FRAME;

            ts.transform.translation.x = local.t[0];
            ts.transform.translation.y = local.t[1];
            ts.transform.translation.z = local.t[2];

            tf2::Matrix3x3 rot(
                local.R[0], local.R[1], local.R[2],
                local.R[3], local.R[4], local.R[5],
                local.R[6], local.R[7], local.R[8]);
            tf2::Quaternion q;
            rot.getRotation(q);

            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();

            broadcaster.sendTransform(ts);
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

// ============================================================

int main(int argc, char **argv)
{
    // =========================================
    // ROS2 初始化 (主线程)
    // =========================================
    rclcpp::init(argc, argv);

    // =========================================
    // USB 相机初始化
    // =========================================
    cv::VideoCapture cap(CAMERA_DEVICE_ID);

    if (!cap.isOpened())
    {
        std::cerr << "无法打开USB相机 /dev/video"
                  << CAMERA_DEVICE_ID << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // =========================================
    // 相机内参矩阵
    // =========================================
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3,3) <<
            FX,  0,  CX,
            0,  FY,  CY,
            0,   0,  1);

    cv::Mat distCoeffs =
        (cv::Mat_<double>(5,1) << K1, K2, P1, P2, K3);


    // =========================================
    // AprilTag 初始化
    // =========================================
    apriltag_family_t *tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma    = QUAD_SIGMA;
    td->nthreads      = NTHREADS;
    td->debug         = 0;
    td->refine_edges  = REFINE_EDGES;

    // =========================================
    // Tag 角点（物体坐标系）
    // =========================================
    float s = TAG_SIZE / 2.0f;

    // SOLVEPNP_IPPE_SQUARE 要求的角点顺序：左上→右上→右下→左下
    std::vector<cv::Point3f> objectPoints =
    {
        {-s,  s, 0},   // 左上
        { s,  s, 0},   // 右上
        { s, -s, 0},   // 右下
        {-s, -s, 0}    // 左下
    };

    // =========================================
    // 窗口
    // =========================================
    cv::namedWindow("AprilTag Pose", cv::WINDOW_NORMAL);
    cv::resizeWindow("AprilTag Pose", WINDOW_WIDTH, WINDOW_HEIGHT);

    // =========================================
    // 启动 ROS2 TF 发布线程
    // =========================================
    std::thread tf_publisher(ros_tf_thread);

    // =========================================
    // 帧平均累加器
    // =========================================
    cv::Mat accum_gray;
    int     acc_count = 0;

    // =========================================
    // 滤波 / 丢帧保持 状态
    // =========================================
    bool            filt_valid = false;
    double          filt_t[3]  = {0.0, 0.0, 0.0};
    tf2::Quaternion filt_q(0.0, 0.0, 0.0, 1.0);
    int             miss_count = 0;
    int             frame_count = 0;       // 帧计数器，用于跳帧打印
    int64           tick_freq  = cv::getTickFrequency();
    int64           tick_start = cv::getTickCount();   // FPS 计时
    int             fps_count  = 0;        // FPS 专用帧计数
    double          fps        = 0.0;

    while (rclcpp::ok())
    {
        // ---- 采集帧 ----
        cv::Mat image;
        if (!cap.read(image) || image.empty()) break;

        // ---- 转灰度 ----
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // ---- 累加 ----
        if (accum_gray.empty())
        {
            gray.convertTo(accum_gray, CV_32F);
        }
        else
        {
            cv::Mat tmp;
            gray.convertTo(tmp, CV_32F);
            accum_gray += tmp;
        }
        acc_count++;

        // 未满 N 帧：只显示原图
        if (acc_count < FRAME_AVG_COUNT)
        {
            cv::imshow("AprilTag Pose", image);
            int key = cv::waitKey(1);
            if (key == 27) break;
            continue;
        }

        // ---- 求平均 ----
        accum_gray /= static_cast<float>(FRAME_AVG_COUNT);
        cv::Mat gray_avg;
        accum_gray.convertTo(gray_avg, CV_8U);

        // 重置累加器
        accum_gray.release();
        acc_count = 0;

        // ---- AprilTag 检测 ----
        image_u8_t img_header =
        {
            .width  = gray_avg.cols,
            .height = gray_avg.rows,
            .stride = gray_avg.cols,
            .buf    = gray_avg.data
        };

        zarray_t *detections =
            apriltag_detector_detect(td, &img_header);

        // ---- 收集本帧所有 pose ----
        std::vector<TagPose> frame_poses;
        int idx_a = -1, idx_b = -1;   // 索引代替指针，避免 vector 扩容导致悬垂

        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // 图像角点：重排成与 objectPoints(IPPE 顺序)一一对应的物理角点
            //   apriltag 角点 p[0..3] 对应 (-s,-s)(s,-s)(s,s)(-s,s)
            //   IPPE 顺序需要 (-s,s)(s,s)(s,-s)(-s,-s) = p[3] p[2] p[1] p[0]
            std::vector<cv::Point2f> imagePoints =
            {
                cv::Point2f(det->p[3][0], det->p[3][1]),
                cv::Point2f(det->p[2][0], det->p[2][1]),
                cv::Point2f(det->p[1][0], det->p[1][1]),
                cv::Point2f(det->p[0][0], det->p[0][1])
            };

            // solvePnP：正方形平面标记用 IPPE_SQUARE，单解稳定、消除翻转
            cv::Mat rvec, tvec;
            bool ok = cv::solvePnP(objectPoints, imagePoints,
                                   cameraMatrix, distCoeffs,
                                   rvec, tvec, false,
                                   cv::SOLVEPNP_IPPE_SQUARE);
            if (!ok) continue;

            // ---- 提取 pose ----
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            TagPose pose;
            pose.id   = det->id;
            pose.t[0] = tvec.at<double>(0);
            pose.t[1] = tvec.at<double>(1);
            pose.t[2] = tvec.at<double>(2);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    pose.R[r*3 + c] = R.at<double>(r, c);

            frame_poses.push_back(pose);

            if (det->id == TAG_ID_A) idx_a = static_cast<int>(frame_poses.size()) - 1;
            if (det->id == TAG_ID_B) idx_b = static_cast<int>(frame_poses.size()) - 1;

            // ---- 控制台输出 ----
            if (frame_count % PRINT_EVERY_N == 0)
            {
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "=== TF tag:" << det->id << " ===\n";
            std::cout << "t: [" << pose.t[0]
                      << ", "    << pose.t[1]
                      << ", "    << pose.t[2] << "]\n";
            std::cout << "R: ["
                      << pose.R[0] << ", " << pose.R[1] << ", " << pose.R[2] << " | "
                      << pose.R[3] << ", " << pose.R[4] << ", " << pose.R[5] << " | "
                      << pose.R[6] << ", " << pose.R[7] << ", " << pose.R[8] << "]\n";
            std::cout << std::endl;
            }

            // ---- 画框 ----
            for (int j = 0; j < 4; j++)
            {
                cv::line(image, imagePoints[j],
                         imagePoints[(j + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
            }

            // ---- 画中心点 ----
            cv::circle(image, cv::Point(det->c[0], det->c[1]),
                       5, cv::Scalar(0, 0, 255), -1);

            // ---- 画坐标轴 ----
            cv::drawFrameAxes(image, cameraMatrix, distCoeffs,
                              rvec, tvec, AXIS_LENGTH);

            // ---- 显示 XYZ ----
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "X:" << pose.t[0]
               << " Y:" << pose.t[1]
               << " Z:" << pose.t[2];

            cv::putText(image, ss.str(),
                        cv::Point(det->c[0], det->c[1] - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 0, 0), 2);

            // ---- Tag ID ----
            cv::putText(image, "ID: " + std::to_string(det->id),
                        cv::Point(det->c[0], det->c[1] + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 2);
        }

        // ---- 融合 + 平滑 + 丢帧保持 → ROS 线程 ----
        MergedPose out;          // 本帧最终要发布的位姿
        out.valid = false;

        if (idx_a >= 0 && idx_b >= 0)
        {
            // ---- 1) 旋转矩阵平均 + SVD 正交化 → R_cam_tag ----
            double Rsum[9];
            for (int i = 0; i < 9; i++)
                Rsum[i] = (frame_poses[idx_a].R[i] + frame_poses[idx_b].R[i]) * 0.5;
            cv::Mat R_avg = (cv::Mat_<double>(3,3) <<
                Rsum[0], Rsum[1], Rsum[2],
                Rsum[3], Rsum[4], Rsum[5],
                Rsum[6], Rsum[7], Rsum[8]);
            cv::SVD svd(R_avg, cv::SVD::FULL_UV);
            cv::Mat R_cam_tag = svd.u * svd.vt;   // 正交化后的 tag 朝向

            // ---- 2) tag 系 → 车体系 的固定旋转 R_tag_car ----
            //   车体 X = tag(-Z)，车体 Y = tag(-X)，车体 Z = tag(+Y)
            cv::Mat R_tag_car = (cv::Mat_<double>(3,3) <<
                 0, -1,  0,
                 0,  0,  1,
                -1,  0,  0);

            // ---- 3) 发布的旋转：R_cam_car = R_cam_tag * R_tag_car ----
            cv::Mat R_cam_car = R_cam_tag * R_tag_car;

            // ---- 4) 平移：平均 tag 中心(相机系) + 车体系偏移旋到相机系 ----
            double px = (frame_poses[idx_a].t[0] + frame_poses[idx_b].t[0]) * 0.5;
            double py = (frame_poses[idx_a].t[1] + frame_poses[idx_b].t[1]) * 0.5;
            double pz = (frame_poses[idx_a].t[2] + frame_poses[idx_b].t[2]) * 0.5;
            cv::Mat off_car = (cv::Mat_<double>(3,1) <<
                OFFSET_X, OFFSET_Y, OFFSET_Z);
            cv::Mat off_cam = R_cam_car * off_car;   // 车体系偏移 → 相机系
            double raw_t[3] = {
                px + off_cam.at<double>(0),
                py + off_cam.at<double>(1),
                pz + off_cam.at<double>(2) };

            // ---- 5) 原始旋转 → 四元数 ----
            tf2::Matrix3x3 m_raw(
                R_cam_car.at<double>(0,0), R_cam_car.at<double>(0,1), R_cam_car.at<double>(0,2),
                R_cam_car.at<double>(1,0), R_cam_car.at<double>(1,1), R_cam_car.at<double>(1,2),
                R_cam_car.at<double>(2,0), R_cam_car.at<double>(2,1), R_cam_car.at<double>(2,2));
            tf2::Quaternion q_raw;
            m_raw.getRotation(q_raw);
            q_raw.normalize();

            // ---- 6) 时间滤波：平移 EMA + 旋转 SLERP ----
            if (!filt_valid)
            {
                for (int i = 0; i < 3; i++) filt_t[i] = raw_t[i];
                filt_q     = q_raw;
                filt_valid = true;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                    filt_t[i] = SMOOTH_ALPHA * raw_t[i] + (1.0 - SMOOTH_ALPHA) * filt_t[i];
                // 取最短路径，避免四元数符号翻转
                if (filt_q.dot(q_raw) < 0.0)
                    q_raw = tf2::Quaternion(-q_raw.x(), -q_raw.y(), -q_raw.z(), -q_raw.w());
                filt_q = filt_q.slerp(q_raw, SMOOTH_ALPHA);
                filt_q.normalize();
            }
            miss_count = 0;
            out.valid  = true;
        }
        else if (filt_valid && miss_count < HOLD_FRAMES)
        {
            // 检测丢失：继续发布上一次滤波后的位姿，避免 TF 闪断
            miss_count++;
            out.valid = true;
        }

        if (out.valid)
        {
            out.t[0] = filt_t[0];
            out.t[1] = filt_t[1];
            out.t[2] = filt_t[2];
            tf2::Matrix3x3 m_out(filt_q);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    out.R[r*3 + c] = m_out[r][c];
            out.x_avg = out.t[0];
            out.z_avg = out.t[2];

            if (frame_count % PRINT_EVERY_N == 0)
            {
            std::cout << std::fixed << std::setprecision(4)
                      << " >>> CAR  X=" << out.t[0]
                      << "  Y=" << out.t[1]
                      << "  Z=" << out.t[2]
                      << ((idx_a >= 0 && idx_b >= 0) ? "" : "  (hold)") << " <<<"
                      << std::endl;
            }
        }

        // ---- 推送给 ROS 线程 ----
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_merged   = out;
            g_new_data = true;
        }
        g_cv.notify_one();

        // ---- 在图像上画发布的车体位置 ----
        if (out.valid)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "CAR X:" << out.t[0]
               << " Y:" << out.t[1]
               << " Z:" << out.t[2]
               << ((idx_a >= 0 && idx_b >= 0) ? "" : " (hold)");
            cv::putText(image, ss.str(),
                        cv::Point(10, image.rows - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }

        // ---- FPS 计算和显示 ----
        {
            int64 tick_now = cv::getTickCount();
            double elapsed  = static_cast<double>(tick_now - tick_start) / tick_freq;
            if (elapsed >= 0.5)   // 每 0.5 秒更新一次 FPS 读数
            {
                fps        = static_cast<double>(fps_count) / elapsed;
                tick_start = tick_now;
                fps_count  = 0;
            }
            std::stringstream sfps;
            sfps << std::fixed << std::setprecision(1) << fps << " fps";
            cv::putText(image, sfps.str(),
                        cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);
        }

        // ---- 显示 ----
        cv::imshow("AprilTag Pose", image);

        if (cv::waitKey(1) == 27) break;

        apriltag_detections_destroy(detections);
        frame_count++;
        fps_count++;
    }

    // =========================================
    // 清理
    // =========================================
    g_running = false;
    g_cv.notify_all();
    if (tf_publisher.joinable())
        tf_publisher.join();

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    rclcpp::shutdown();

    return 0;
}