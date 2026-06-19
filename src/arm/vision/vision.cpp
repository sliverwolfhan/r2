<<<<<<< Updated upstream
// #include <iostream>
// #include <vector>
// #include <array>
// #include <cmath>
// #include <algorithm>
// #include <deque>
// #include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

// using namespace std;
// using namespace cv;
// using namespace cv::dnn;

// // ================================================================
// // 配置
// // ================================================================
// // 32种图案物理尺寸 (宽,高) mm, 索引 = YOLO class_id
// static const float PS[32][2] = {
//     {185,330},{185,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},
//     {300,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},
//     {300,330},{185,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},
//     {300,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},{300,330},
// };

// static vector<Point3f> makeObjPts(int cls) {
//     int i = clamp(cls, 0, 31);
//     float hw = PS[i][0] / 2.f, hh = PS[i][1] / 2.f;
//     return { {-hw,-hh,0}, {hw,-hh,0}, {hw,hh,0}, {-hw,hh,0} };
// }

// static const int   SMOOTH_N   = 6;
// static const char* MODEL_PATH = "/home/hao/arm/best.onnx";
// static const int   YOLO_W     = 640, YOLO_H = 640;
// static const float YOLO_CONF  = 0.25f, YOLO_NMS = 0.45f;
// static const int   YOLO_NC    = 32;
// static const char* CAM_FRAME  = "camera_link";
// static const char* OBJ_FRAME  = "target_object";

// // ================================================================
// // CornerKF
// // ================================================================
// struct CornerKF {
//     KalmanFilter kf;
//     bool ok = false;

//     void init(Point2f pt) {
//         kf.init(4, 2, 0, CV_64F);
//         setIdentity(kf.transitionMatrix);
//         kf.transitionMatrix.at<double>(0,2) = 1;
//         kf.transitionMatrix.at<double>(1,3) = 1;
//         kf.measurementMatrix = Mat::zeros(2, 4, CV_64F);
//         kf.measurementMatrix.at<double>(0,0) = 1;
//         kf.measurementMatrix.at<double>(1,1) = 1;
//         setIdentity(kf.processNoiseCov,     Scalar(0.05));
//         setIdentity(kf.measurementNoiseCov, Scalar(0.8));
//         setIdentity(kf.errorCovPost,        Scalar(1));
//         kf.statePost = (Mat_<double>(4,1) << pt.x, pt.y, 0, 0);
//         ok = true;
//     }
//     Point2f update(Point2f m) {
//         if (!ok) init(m);
//         kf.predict();
//         Mat meas = (Mat_<double>(2,1) << m.x, m.y);
//         Mat est  = kf.correct(meas);
//         return Point2f((float)est.at<double>(0), (float)est.at<double>(1));
//     }
//     Point2f predictOnly() {
//         Mat p = kf.predict();
//         return Point2f((float)p.at<double>(0), (float)p.at<double>(1));
//     }
// };

// // ================================================================
// // 工具
// // ================================================================
// Vec3d euler(const Mat& R) {
//     double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0)
//                    + R.at<double>(1,0)*R.at<double>(1,0));
//     if (sy < 1e-6)
//         return Vec3d(atan2(-R.at<double>(1,2), R.at<double>(1,1)) * 180/CV_PI,
//                      atan2(-R.at<double>(2,0), sy) * 180/CV_PI, 0);
//     return Vec3d(atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180/CV_PI,
//                  atan2(-R.at<double>(2,0), sy) * 180/CV_PI,
//                  atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180/CV_PI);
// }

// void putLabel(Mat& im, const string& t, Point o,
//               double s = 0.65, Scalar c = Scalar(0,255,255), int th = 2) {
//     putText(im, t, o, FONT_HERSHEY_SIMPLEX, s, Scalar(0,0,0), th+2);
//     putText(im, t, o, FONT_HERSHEY_SIMPLEX, s, c, th);
// }

// // ================================================================
// // YOLO
// // ================================================================
// struct YoloResult { Rect box; int cls = 0; };

// YoloResult yoloDetect(const Mat& fr, Net& net) {
//     Mat blob;
//     blobFromImage(fr, blob, 1.0/255.0, Size(YOLO_W,YOLO_H), Scalar(), true, false);
//     net.setInput(blob);
//     vector<Mat> outs;
//     net.forward(outs, net.getUnconnectedOutLayersNames());
//     if (outs.empty()) return {Rect(), -1};

//     Mat out = outs[0];
//     int nb = out.size[2];
//     float* d = (float*)out.data;
//     float xf = (float)fr.cols / YOLO_W, yf = (float)fr.rows / YOLO_H;

//     vector<Rect> bx; vector<float> cf; vector<int> cl;
//     for (int i = 0; i < nb; i++) {
//         float cx = d[0*nb+i], cy = d[1*nb+i], w = d[2*nb+i], h = d[3*nb+i];
//         float bs = 0; int bc = -1;
//         for (int c = 0; c < YOLO_NC; c++) {
//             float s = d[(4+c)*nb+i]; if (s > bs) { bs = s; bc = c; }
//         }
//         if (bs < YOLO_CONF) continue;
//         bx.emplace_back(int((cx-w*.5f)*xf), int((cy-h*.5f)*yf), int(w*xf), int(h*yf));
//         cf.push_back(bs); cl.push_back(bc);
//     }
//     if (bx.empty()) return {Rect(), -1};

//     vector<int> idx;
//     NMSBoxes(bx, cf, YOLO_CONF, YOLO_NMS, idx);
//     if (idx.empty()) return {Rect(), -1};

//     int best = idx[0];
//     for (int i : idx) if (cf[i] > cf[best]) best = i;
//     return {bx[best], cl[best]};
// }

// static vector<Point2f> boxCorners(const Rect& r) {
//     return {Point2f(r.x,r.y), Point2f(r.x+r.width,r.y),
//             Point2f(r.x+r.width,r.y+r.height), Point2f(r.x,r.y+r.height)};
// }

// // ================================================================
// // 指数移动平均平滑 (避免中值滤波的死值问题)
// // ================================================================
// struct Ema3 {
//     double vx = 0, vy = 0, vz = 0;
//     bool   initd = false;
//     static constexpr double A = 0.35;   // 平滑系数 (越小越平滑)

//     Vec3d push(double x, double y, double z) {
//         if (!initd) { vx = x; vy = y; vz = z; initd = true; }
//         else { vx += A * (x - vx); vy += A * (y - vy); vz += A * (z - vz); }
//         return Vec3d(vx, vy, vz);
//     }
//     void reset() { initd = false; }
// };

// // ================================================================
// // main
// // ================================================================
// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto nd = rclcpp::Node::make_shared("vision_node");
//     auto tb = std::make_shared<tf2_ros::TransformBroadcaster>(nd);

//     // ---- RealSense ----
//     rs2::pipeline pp;
//     rs2::config   cfg;
//     cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
//     cfg.enable_stream(RS2_STREAM_DEPTH, 848,  480, RS2_FORMAT_Z16,  30);

//     rs2::pipeline_profile pr;
//     try { pr = pp.start(cfg); }
//     catch (const rs2::error& e) {
//         cerr << "RealSense: " << e.what() << endl;
//         rclcpp::shutdown(); return -1;
//     }
//     rs2::align al2c(RS2_STREAM_COLOR);
//     float ds = pr.get_device().first<rs2::depth_sensor>().get_depth_scale();
//     auto cpr = pr.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
//     rs2_intrinsics ci = cpr.get_intrinsics();

//     Mat K = (Mat_<double>(3,3) << ci.fx, 0, ci.ppx, 0, ci.fy, ci.ppy, 0, 0, 1);
//     Mat D = Mat::zeros(1, 5, CV_64F);
//     double fx = ci.fx, fy = ci.fy, cx = ci.ppx, cy = ci.ppy;
//     cout << "K:\n" << K << "  scale=" << ds << endl;

//     // ---- YOLO ----
//     Net net = readNet(MODEL_PATH);
//     if (net.empty()) { cerr << "YOLO fail" << endl; pp.stop(); rclcpp::shutdown(); return -1; }
//     net.setPreferableBackend(DNN_BACKEND_OPENCV);
//     net.setPreferableTarget(DNN_TARGET_CPU);
//     cout << "YOLO loaded\n";

//     // ---- 状态 ----
//     array<CornerKF, 4> kfs;
//     Ema3               sm;               // XYZ 指数平滑
//     Rect               lastBox;
//     bool               live = false;

//     // 面板显示值（每帧都更新）
//     double dX = 0, dY = 0, dZ_px = 0, dDist = 0;
//     double dRoll = 0, dPitch = 0, dYaw = 0;
//     bool   havePose = false;
//     bool   haveRpy  = false;

//     while (rclcpp::ok()) {
//         rclcpp::spin_some(nd);

//         // ---- 取帧 ----
//         rs2::frameset fs;
//         try { fs = pp.wait_for_frames(); fs = al2c.process(fs); }
//         catch (const rs2::error&) { break; }
//         rs2::video_frame cf = fs.get_color_frame();
//         rs2::depth_frame df = fs.get_depth_frame();
//         if (!cf) break;

//         Mat bgr(Size(ci.width, ci.height), CV_8UC3,
//                 (void*)cf.get_data(), Mat::AUTO_STEP);
//         bgr = bgr.clone();
//         Mat depth;
//         if (df) {
//             depth = Mat(Size(ci.width, ci.height), CV_16UC1,
//                         (void*)df.get_data(), Mat::AUTO_STEP);
//             depth = depth.clone();
//         }
//         Mat show = bgr.clone();

//         // ---- Step 1: YOLO ----
//         YoloResult yr = yoloDetect(bgr, net);
//         live = !yr.box.empty();
//         if (!live) {
//             // 没检测到：重置 KF，只显示画面，不发 TF，不预测
//             for (auto& kf : kfs) kf.ok = false;
//             sm.reset();
//             imshow("Box PnP", show);
//             char key = (char)waitKey(1);
//             if (key == 27) break;
//             if (key == 'r') lastBox = Rect();
//             continue;
//         }
//         int cls = (yr.cls >= 0) ? yr.cls : 0;
//         auto objPts = makeObjPts(cls);

//         // ---- Step 2: KF 平滑角点 ----
//         vector<Point2f> raw = boxCorners(yr.box);
//         vector<Point2f> smCorners(4);
//         for (int i = 0; i < 4; i++) smCorners[i] = kfs[i].update(raw[i]);

//         // ---- Step 3: 深度 Z + 针孔 X,Y + PnP 旋转 ----
//         bool pnp_ok = false;
//         double z_mm = 0;
//         haveRpy = false;

//         if (kfs[0].ok) {
//             // 3a. 读深度中值
//             int bpx = yr.box.x + yr.box.width/2, bpy = yr.box.y + yr.box.height/2;
//             bpx = clamp(bpx, 0, bgr.cols-1); bpy = clamp(bpy, 0, bgr.rows-1);
//             if (!depth.empty()) {
//                 vector<uint16_t> dv;
//                 const int R = 7;
//                 for (int dy = -R; dy <= R; dy++)
//                     for (int dx = -R; dx <= R; dx++) {
//                         int sx = clamp(bpx+dx, 0, bgr.cols-1);
//                         int sy = clamp(bpy+dy, 0, bgr.rows-1);
//                         uint16_t v = depth.at<uint16_t>(sy, sx);
//                         if (v > 0) dv.push_back(v);
//                     }
//                 if (!dv.empty()) {
//                     size_t mid = dv.size()/2;
//                     nth_element(dv.begin(), dv.begin()+mid, dv.end());
//                     z_mm = (double)dv[mid] * ds * 1000.0;
//                 }
//             }

//             // 3b. 针孔反投影
//             double ppx = yr.box.x + yr.box.width/2.0;
//             double ppy = yr.box.y + yr.box.height/2.0;
//             double Zmm = (z_mm > 0) ? z_mm : 500.0;
//             double Xmm = (ppx - cx) * Zmm / fx;
//             double Ymm = (ppy - cy) * Zmm / fy;

//             // 指数平滑（无死值问题）
//             Vec3d sv = sm.push(Xmm, Ymm, Zmm);
//             dX = sv[0]; dY = sv[1]; dZ_px = sv[2];
//             dDist = sqrt(dX*dX + dY*dY + dZ_px*dZ_px);
//             havePose = true;

//             // 3c. PnP → 旋转 + TF
//             if (z_mm > 0) {   // 深度有效才做 PnP
//                 Mat rvec, tvec;
//                 pnp_ok = solvePnP(objPts, smCorners, K, D, rvec, tvec, false, SOLVEPNP_IPPE);
//                 if (pnp_ok) {
//                     Mat R; Rodrigues(rvec, R);
//                     Vec3d eu = euler(R);
//                     dRoll = eu[0]; dPitch = eu[1]; dYaw = eu[2];
//                     haveRpy = true;

//                     // TF: PnP 成功即发
//                     {
//                         geometry_msgs::msg::TransformStamped tf;
//                         tf.header.stamp    = nd->get_clock()->now();
//                         tf.header.frame_id = CAM_FRAME;
//                         tf.child_frame_id  = OBJ_FRAME;
//                         // ROS 坐标系: X前 Y左 Z上  ← 相机: X右 Y下 Z前
//                         tf.transform.translation.x = dZ_px / 1000.0;   // Z前 → X
//                         tf.transform.translation.y = -dX   / 1000.0;   // X右 → -Y
//                         tf.transform.translation.z = -dY   / 1000.0;   // Y下 → -Z
//                         tf2::Matrix3x3 mr(
//                             R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
//                             R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
//                             R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2));
//                         tf2::Quaternion q; mr.getRotation(q);
//                         tf.transform.rotation.x = q.x();
//                         tf.transform.rotation.y = q.y();
//                         tf.transform.rotation.z = q.z();
//                         tf.transform.rotation.w = q.w();
//                         tb->sendTransform(tf);
//                         printf("\r>>> TF sent! (cam XYZ=%.0f %.0f %.0f mm)                              \n", dX, dY, dZ_px);
//                     }

//                     // 坐标轴
//                     Point2f ao = (smCorners[0]+smCorners[1]+smCorners[2]+smCorners[3])*0.25f;
//                     Mat tvm = (Mat_<double>(3,1) << dX, dY, dZ_px);
//                     vector<Point3f> ax3 = {{0,0,0},{100,0,0},{0,100,0},{0,0,100}};
//                     vector<Point2f> ax2;
//                     projectPoints(ax3, rvec, tvm, K, D, ax2);
//                     Point2f vx = ax2[1]-ax2[0], vy = ax2[2]-ax2[0], vz = ax2[3]-ax2[0];
//                     const float AL = 50;
//                     float nx = norm(vx), ny = norm(vy), nz = norm(vz);
//                     if (nx>1e-3f) vx *= AL/nx;
//                     if (ny>1e-3f) vy *= AL/ny;
//                     if (nz>1e-3f) vz *= AL/nz;
//                     arrowedLine(show, ao, ao+vx, Scalar(0,0,255),   3, LINE_AA, 0, 0.2);
//                     arrowedLine(show, ao, ao+vy, Scalar(0,255,0),   3, LINE_AA, 0, 0.2);
//                     arrowedLine(show, ao, ao+vz, Scalar(255,0,0),   3, LINE_AA, 0, 0.2);
//                     putLabel(show, "X", ao+vx+Point2f(5,0), 0.6, Scalar(0,0,255));
//                     putLabel(show, "Y", ao+vy+Point2f(5,0), 0.6, Scalar(0,255,0));
//                     putLabel(show, "Z", ao+vz+Point2f(5,0), 0.6, Scalar(255,100,0));
//                     line(show, Point((int)ao.x-14,(int)ao.y), Point((int)ao.x+14,(int)ao.y), Scalar(0,255,255),2);
//                     line(show, Point((int)ao.x,(int)ao.y-14), Point((int)ao.x,(int)ao.y+14), Scalar(0,255,255),2);
//                     circle(show, ao, 5, Scalar(0,255,255), -1);

//                     printf("\rD=%.0f XYZ=[%.0f %.0f %.0f]mm rawZ=%.0f Zmm=%.0f RPY=[%.1f %.1f %.1f]   ",
//                            dDist, dX, dY, dZ_px, z_mm, Zmm, dRoll, dPitch, dYaw);
//                     fflush(stdout);
//                 }
//             }
//             if (!pnp_ok) {
//                 printf("\rD=%.0f XYZ=[%.0f %.0f %.0f]mm rawZ=%.0f RPY=---         ",
//                        dDist, dX, dY, dZ_px, z_mm);
//                 fflush(stdout);
//             }
//         }

//         // ---- 画框 + 角点 + ID ----
//         {
//             Scalar bc = pnp_ok ? Scalar(0,220,0) : Scalar(0,140,255);
//             rectangle(show, yr.box, bc, 2);
//             for (int i = 0; i < 4; i++)
//                 circle(show, smCorners[i], 5, Scalar(0,0,255), -1);
//             char tag[64];
//             sprintf(tag, "ID=%d %.0fx%.0f", cls, PS[cls][0], PS[cls][1]);
//             putLabel(show, tag, Point(yr.box.x, yr.box.y-8), 0.45, Scalar(0,255,255));
//         }

//         // ---- 面板 ----
//         if (havePose) {
//             Mat ov = show.clone();
//             rectangle(ov, Point(8,8), Point(340,110), Scalar(0,0,0), FILLED);
//             addWeighted(ov, 0.45, show, 0.55, 0, show);
//             int bx = 18, by = 30, dy = 25;
//             char buf[128];
//             sprintf(buf, "Dist: %.0f mm", dDist);
//             putLabel(show, buf, Point(bx,by), 0.65, Scalar(0,255,100), 2);
//             sprintf(buf, "XYZ: %.0f  %.0f  %.0f mm", dX, dY, dZ_px);
//             putLabel(show, buf, Point(bx,by+dy), 0.55, Scalar(0,220,255));
//             if (haveRpy)
//                 sprintf(buf, "RPY: %.1f  %.1f  %.1f deg", dRoll, dPitch, dYaw);
//             else
//                 sprintf(buf, "RPY: --- (no PnP)");
//             putLabel(show, buf, Point(bx,by+dy*2), 0.55, Scalar(255,200,0));
//         }

//         imshow("Box PnP", show);
//         char key = (char)waitKey(1);
//         if (key == 27) break;
//         if (key == 'r') {
//             for (auto& kf : kfs) kf.ok = false;
//             sm.reset(); lastBox = Rect();
//         }
//     }
//     pp.stop(); destroyAllWindows(); rclcpp::shutdown(); return 0;
// }
=======
// #include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>
// #include <iostream>
// #include <vector>

// using namespace cv;
// using namespace cv::dnn;
// using namespace std;

// static const vector<string> class_names =
// {
//     "R_R1",
//     "B_R1",
//     "T_03",
//     "T_04",
//     "T_05",
//     "T_06",
//     "T_07",
//     "T_08",
//     "T_09",
//     "T_10",
//     "T_11",
//     "T_12",
//     "T_13",
//     "T_14",
//     "T_15",
//     "T_16",
//     "T_17",
//     "F_18",
//     "F_19",
//     "F_20",
//     "F_21",
//     "F_22",
//     "F_23",
//     "F_24",
//     "F_25",
//     "F_26",
//     "F_27",
//     "F_28",
//     "F_29",
//     "F_30",
//     "F_31",
//     "F_32"
// };

// int main()
// {
//     string model_path = "/home/hao/arm/best.onnx";

//     Net net = readNet(model_path);

//     if(net.empty())
//     {
//         cerr << "加载模型失败: "
//              << model_path
//              << endl;
//         return -1;
//     }

//     net.setPreferableBackend(DNN_BACKEND_OPENCV);
//     net.setPreferableTarget(DNN_TARGET_CPU);

//     VideoCapture cap(2);

//     if(!cap.isOpened())
//     {
//         cerr << "打开摄像头失败" << endl;
//         return -1;
//     }

//     cap.set(CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(CAP_PROP_FRAME_HEIGHT, 480);

//     const int INPUT_W = 640;
//     const int INPUT_H = 640;

//     while(true)
//     {
//         Mat frame;
//         cap >> frame;

//         if(frame.empty())
//             continue;

//         Mat blob;

//         blobFromImage(
//             frame,
//             blob,
//             1.0 / 255.0,
//             Size(INPUT_W, INPUT_H),
//             Scalar(),
//             true,
//             false
//         );

//         net.setInput(blob);

//         vector<Mat> outputs;

//         net.forward(
//             outputs,
//             net.getUnconnectedOutLayersNames()
//         );

//         if(outputs.empty())
//         {
//             cout << "推理输出为空" << endl;
//             continue;
//         }

//         Mat output = outputs[0];

//         vector<int> class_ids;
//         vector<float> confidences;
//         vector<Rect> boxes;

//         const int num_classes = 32;
//         const int num_boxes = output.size[2];

//         float* data =
//             (float*)output.data;

//         float x_factor =
//             frame.cols / 640.0f;

//         float y_factor =
//             frame.rows / 640.0f;

//         for(int i = 0; i < num_boxes; i++)
//         {
//             float cx =
//                 data[0 * num_boxes + i];

//             float cy =
//                 data[1 * num_boxes + i];

//             float w =
//                 data[2 * num_boxes + i];

//             float h =
//                 data[3 * num_boxes + i];

//             float max_score = 0.0f;
//             int class_id = -1;

//             for(int c = 0;
//                 c < num_classes;
//                 c++)
//             {
//                 float score =
//                     data[(4 + c)
//                     * num_boxes + i];

//                 if(score > max_score)
//                 {
//                     max_score = score;
//                     class_id = c;
//                 }
//             }

//             if(max_score < 0.25f)
//                 continue;

//             int left =
//                 int((cx - w * 0.5f)
//                 * x_factor);

//             int top =
//                 int((cy - h * 0.5f)
//                 * y_factor);

//             int width =
//                 int(w * x_factor);

//             int height =
//                 int(h * y_factor);

//             boxes.emplace_back(
//                 left,
//                 top,
//                 width,
//                 height
//             );

//             confidences.push_back(
//                 max_score
//             );

//             class_ids.push_back(
//                 class_id
//             );
//         }

//         vector<int> indices;

//         NMSBoxes(
//             boxes,
//             confidences,
//             0.25,
//             0.45,
//             indices
//         );

//         for(int idx : indices)
//         {
//             Rect box = boxes[idx];

//             rectangle(
//                 frame,
//                 box,
//                 Scalar(0,255,0),
//                 2
//             );

//             string label;

//             if(class_ids[idx] >= 0 &&
//                class_ids[idx] < (int)class_names.size())
//             {
//                 label =
//                     class_names[
//                         class_ids[idx]
//                     ]
//                     + " "
//                     + format(
//                         "%.2f",
//                         confidences[idx]
//                     );
//             }
//             else
//             {
//                 label =
//                     "unknown "
//                     + format(
//                         "%.2f",
//                         confidences[idx]
//                     );
//             }

//             putText(
//                 frame,
//                 label,
//                 Point(
//                     box.x,
//                     max(
//                         20,
//                         box.y - 5
//                     )
//                 ),
//                 FONT_HERSHEY_SIMPLEX,
//                 0.6,
//                 Scalar(0,255,0),
//                 2
//             );
//         }

//         putText(
//             frame,
//             "Detections: "
//             + to_string(indices.size()),
//             Point(10,30),
//             FONT_HERSHEY_SIMPLEX,
//             0.8,
//             Scalar(0,0,255),
//             2
//         );

//         imshow(
//             "YOLO11 ONNX",
//             frame
//         );

//         char key =
//             (char)waitKey(1);

//         if(key == 27)
//             break;
//     }

//     cap.release();
//     destroyAllWindows();

//     return 0;
// }
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

>>>>>>> Stashed changes
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace cv;
using namespace cv::dnn;

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
 * YOLO 模型参数
 */
static const char*  MODEL_PATH     = "/home/pc1/AT_RC/src/arm/vision/best.onnx";
static const int    YOLO_INPUT_W   = 640;
static const int    YOLO_INPUT_H   = 640;
static const float  YOLO_CONF_THR  = 0.25f;
static const float  YOLO_NMS_THR   = 0.45f;
static const float  ROI_EXPAND_X   = 2.5f;        // YOLO框横向扩大倍数
static const float  ROI_EXPAND_Y   = 2.0f;        // YOLO框纵向扩大倍数（1=原框, 3=三倍）
static const int    YOLO_NUM_CLASSES = 32;

// YOLO 类别名（仅 R_R1=0 和 B_R1=1 是我们关心的红色/蓝色箱子）
static const vector<string> YOLO_CLASS_NAMES =
{
    "R_R1","B_R1","T_03","T_04","T_05","T_06","T_07","T_08",
    "T_09","T_10","T_11","T_12","T_13","T_14","T_15","T_16",
    "T_17","F_18","F_19","F_20","F_21","F_22","F_23","F_24",
    "F_25","F_26","F_27","F_28","F_29","F_30","F_31","F_32"
};

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

        568.158366, 0.0, 666.658980,
        0.0, 563.828657, 360.392824,
        0.0, 0.0, 1.0);
/**
 * 相机畸变系数 D
 * 格式：[k1, k2, p1, p2, k3]
 * k1, k2, k3: 径向畸变系数
 * p1, p2: 切向畸变系数
 */
static const Mat D = (Mat_<double>(1,5) <<

        -0.039200,
        -0.011498,
         0.001678,
        -0.006394,
         0.0);



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
//  YOLO 粗定位：检测红/蓝箱子，返回最大置信度的 bounding box
// ================================================================
/**
 * @brief 用 YOLO11 ONNX 模型检测红色/蓝色箱子顶面
 * @param frame 输入图像（BGR，任意尺寸）
 * @param net   YOLO ONNX 网络
 * @param yoloBox 输出检测框（原图坐标），检测失败返回空 Rect
 */
Rect runYoloDetection(const Mat& frame, Net& net)
{
    // cout << "1" << endl;

    // ---- 1. 构造 blob ----
    Mat blob;
    blobFromImage(frame, blob, 1.0/255.0, Size(YOLO_INPUT_W, YOLO_INPUT_H),
                  Scalar(), true, false);
    // cout << "2" << endl;

    net.setInput(blob);
    // cout << "3" << endl;

    // ---- 2. 推理 ----
    vector<Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return Rect();
    // cout << "4" << endl;

    Mat output = outputs[0];
    const int numBoxes = output.size[2];           // e.g. 8400
    float* data = (float*)output.data;

    float xFactor = (float)frame.cols / YOLO_INPUT_W;
    float yFactor = (float)frame.rows / YOLO_INPUT_H;

    // ---- 3. 解析所有检测框，收集 R_R1(0) 和 B_R1(1) ----
    vector<Rect>   boxes;
    vector<float>  confs;
    vector<int>    clsIds;

    for (int i = 0; i < numBoxes; i++)
    {
        float cx = data[0 * numBoxes + i];
        float cy = data[1 * numBoxes + i];
        float w  = data[2 * numBoxes + i];
        float h  = data[3 * numBoxes + i];

        // 找最佳类别
        float bestScore = 0;
        int   bestClass = -1;
        for (int c = 0; c < YOLO_NUM_CLASSES; c++)  // 检测全部类别
        {
            float score = data[(4 + c) * numBoxes + i];
            if (score > bestScore) { bestScore = score; bestClass = c; }
        }

        if (bestScore < YOLO_CONF_THR) continue;

        int left   = int((cx - w * 0.5f) * xFactor);
        int top    = int((cy - h * 0.5f) * yFactor);
        int width  = int(w * xFactor);
        int height = int(h * yFactor);

        boxes.emplace_back(left, top, width, height);
        confs.push_back(bestScore);
        clsIds.push_back(bestClass);
    }

    if (boxes.empty()) return Rect();

    // ---- 4. NMS ----
    vector<int> indices;
    NMSBoxes(boxes, confs, YOLO_CONF_THR, YOLO_NMS_THR, indices);
    if (indices.empty()) return Rect();

    // ---- 5. 返回置信度最高的那个框 ----
    int bestIdx = indices[0];
    for (int idx : indices)
        if (confs[idx] > confs[bestIdx]) bestIdx = idx;

    return boxes[bestIdx];
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
    // ---- 1. HSV 红色分割（自适应光照） ----
    // 转换到 HSV 颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // ---- 对 V 通道做 CLAHE 自适应直方图均衡，减少光照变化影响 ----
    {
        vector<Mat> hsvChannels;
        split(hsv, hsvChannels);                    // H[0], S[1], V[2]
        Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8,8));  // clipLimit=2.0, tile=8x8
        clahe->apply(hsvChannels[2], hsvChannels[2]);     // 只对 V 通道做均衡
        merge(hsvChannels, hsv);
    }

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

    // 检查角点是否在图像范围内，越界的角点说明检测不可靠，直接丢弃本帧
    bool anyClamped = false;
    for (auto& p : raw)
    {
        if (p.x < 0.f || p.x > (float)(frame.cols - 1) ||
            p.y < 0.f || p.y > (float)(frame.rows - 1))
        {
            anyClamped = true;
        }
        p.x = clamp(p.x, 0.f, (float)(frame.cols - 1));
        p.y = clamp(p.y, (float)0, (float)(frame.rows - 1));
    }
    if (anyClamped) return false;  // 角点越界，本帧不可靠
    
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
 * @brief 对位姿向量进行中值滤波 + 异常剔除平滑
 *
 * 作用：减少位姿抖动，同时自动剔除异常值，使输出更稳定
 */
struct PoseSmootherVec3
{
    deque<Vec3d> buf;   // 历史数据缓冲区
    int maxN;           // 最大帧数

    explicit PoseSmootherVec3(int n) : maxN(n) {}

    /**
     * @brief 添加新数据并返回中值平滑结果（自动剔除异常值）
     * @param v 新的位姿向量
     * @return 平滑后的位姿向量（各分量取中位数）
     */
    Vec3d push(Vec3d v)
    {
        // 异常值剔除：缓冲区已有足够数据时，先检查新值是否离群
        if ((int)buf.size() >= 3)
        {
            Vec3d curMed = median();
            double dev = norm(v - curMed);
            // 偏离当前中值超过 200mm 视为异常值，直接丢弃
            static const double OUTLIER_THRESH_MM = 200.0;
            if (dev > OUTLIER_THRESH_MM) return curMed;
        }

        buf.push_back(v);
        if ((int)buf.size() > maxN) buf.pop_front();  // 保持窗口大小

        return median();
    }

    /**
     * @brief 取各分量的中位数
     */
    Vec3d median() const
    {
        int n = (int)buf.size();
        if (n == 0) return Vec3d(0,0,0);

        vector<double> xs(n), ys(n), zs(n);
        for (int i = 0; i < n; i++) {
            xs[i] = buf[i][0];
            ys[i] = buf[i][1];
            zs[i] = buf[i][2];
        }

        int mid = n / 2;
        nth_element(xs.begin(), xs.begin() + mid, xs.end());
        nth_element(ys.begin(), ys.begin() + mid, ys.end());
        nth_element(zs.begin(), zs.begin() + mid, zs.end());

        return Vec3d(xs[mid], ys[mid], zs[mid]);
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

    // ---------- 启动 USB 摄像头（写死路径，插拔不变） ----------
    // 插上相机后运行: ls /dev/v4l/by-id/usb-*video* | head -1
    // 把输出填到下面 CAMERA_PATH
    static const char* CAMERA_PATH = "/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0";

    VideoCapture cap;
    bool opened = cap.open(CAMERA_PATH, CAP_V4L2);
    if (opened)
    {
        cout << "摄像头打开成功: " << CAMERA_PATH << endl;
    }
    else
    {
        cerr << "无法打开摄像头: " << CAMERA_PATH
             << "\n请运行: ls /dev/v4l/by-id/usb-*video* | head -1"
             << "\n然后把结果填入 CAMERA_PATH\n";
    }


    if (!opened || !cap.isOpened()) {
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

    // ========== 2.5 加载 YOLO ONNX 模型 ==========
    Net yoloNet = readNet(MODEL_PATH);
    if (yoloNet.empty())
    {
        cerr << "加载 YOLO 模型失败: " << MODEL_PATH << endl;
        cap.release();
        rclcpp::shutdown();
        return -1;
    }
    yoloNet.setPreferableBackend(DNN_BACKEND_OPENCV);
    yoloNet.setPreferableTarget(DNN_TARGET_CPU);
    cout << "[INFO] YOLO model loaded: " << MODEL_PATH << endl;

    // ========== 3. 初始化滤波器 ==========
    // 卡尔曼滤波器：4 个角点各一个
    array<CornerKF, 4> kfs;
    
    // 位姿平滑器
    PoseSmootherVec3 tvecSmoother(SMOOTH_N);
    
    // 上一帧灰度图（预留，当前未使用）
    Mat prevGray;
    bool hasPrev = false;

    // 连续丢帧计数器（超过上限则复位卡尔曼滤波器并停止发布TF）
    int consecutiveLost = 0;
    static const int MAX_LOST_FRAMES = 10;

<<<<<<< Updated upstream
=======
    // 上一帧有效距离（用于帧间跳变检测）
    double prevValidDist = -1.0;

>>>>>>> Stashed changes
    // 零畸变系数（用于去畸变后的图像）
    Mat zeroDist = Mat::zeros(1, 5, CV_64F);

    // 上一帧 YOLO 检测框（YOLO 丢帧时复用）
    Rect lastYoloBox;

    // 全局平滑位姿（面板始终显示，不受 pnp_ok/detected 影响）
    double gX = 0, gY = 0, gZ = 0, gDist = 0;
    double gRoll = 0, gPitch = 0, gYaw = 0;
    bool   gHavePose = false;

    // ========== 4. 主循环 ==========
    // 流程：YOLO粗定位 → 框扩大3x → 裁剪ROI → HSV精提取角点 → KF平滑 → PnP → TF
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
        Mat debugMask = Mat::zeros(undistorted.size(), CV_8UC1);
        vector<Point2f> rawCorners;

        // ----- Step 1: YOLO 粗定位 -----
        Rect yoloBox = runYoloDetection(undistorted, yoloNet);

        // YOLO 丢帧时复用上一帧框
        if (yoloBox.empty()) yoloBox = lastYoloBox;
        else                 lastYoloBox = yoloBox;

        // ----- Step 2: 在 YOLO 框扩大 3x 的 ROI 内做 HSV 精提取 -----
        bool detected = false;
        Rect roi;
        bool hasRoi = false;
        if (!yoloBox.empty())
        {
            // 扩大 ROI_EXPAND 倍，clamp 到图像边界
            int cx = yoloBox.x + yoloBox.width / 2;
            int cy = yoloBox.y + yoloBox.height / 2;
            int halfW = (int)(yoloBox.width  * ROI_EXPAND_X / 2.0f);
            int halfH = (int)(yoloBox.height * ROI_EXPAND_Y / 2.0f);
            int roiX = max(0, cx - halfW);
            int roiY = max(0, cy - halfH);
            int roiW = min(undistorted.cols - roiX, halfW * 2);
            int roiH = min(undistorted.rows - roiY, halfH * 2);
            roi = Rect(roiX, roiY, roiW, roiH);
            hasRoi = true;

            // 裁剪图像送给 HSV 检测
            Mat crop = undistorted(roi);
            Mat cropMask;
            vector<Point2f> cropCorners;
            bool cropDetected = detectBoxCorners(crop, prevGray, cropCorners, cropMask);

            if (cropDetected)
            {
                // 角点从 crop 坐标 → 原图坐标
                for (auto& p : cropCorners)
                {
                    p.x += (float)roiX;
                    p.y += (float)roiY;
                }
                rawCorners = cropCorners;
                detected = true;

                // mask 从 crop 坐标 → 原图坐标（嵌入全图 debugMask）
                cropMask.copyTo(debugMask(roi));
            }
        }

        // YOLO 彻底不可用时回退全帧 HSV
        if (!detected && yoloBox.empty())
        {
            detected = detectBoxCorners(undistorted, prevGray, rawCorners, debugMask);
        }

        // 在 show 图上画 YOLO 原始框 + 扩大的 ROI 框
        if (!yoloBox.empty())
        {
            rectangle(show, yoloBox, Scalar(255, 150, 0), 2, LINE_AA);  // 橙-原框
            if (hasRoi)
                rectangle(show, roi, Scalar(0, 255, 255), 1, LINE_AA);  // 黄-扩大ROI
        }

        // ----- 卡尔曼平滑角点 -----
        vector<Point2f> smoothCorners(4);
        if (detected)
        {
            consecutiveLost = 0;  // 检测成功，复位丢帧计数
            // 检测成功：使用测量值更新卡尔曼滤波器
            for (int i = 0; i < 4; i++)
                smoothCorners[i] = kfs[i].update(rawCorners[i]);
        }
        else
        {
            consecutiveLost++;
            // 检测失败：使用卡尔曼滤波器预测
            for (int i = 0; i < 4; i++)
            {
                if (kfs[i].initialized)
                    smoothCorners[i] = kfs[i].predictOnly();
                else
                    smoothCorners[i] = Point2f(0,0);
            }
        }

        // 连续丢帧过多：复位卡尔曼滤波器，防止预测值漂移
        bool kfTimedOut = (consecutiveLost > MAX_LOST_FRAMES);
        if (kfTimedOut)
<<<<<<< Updated upstream
=======
        {
            for (auto& kf : kfs) kf.initialized = false;
            tvecSmoother.buf.clear();
            consecutiveLost = 0;
            prevValidDist = -1.0;
            cerr << "\n[WARN] KF timed out after " << MAX_LOST_FRAMES
                 << " consecutive lost frames, resetting filters.\n";
        }

        bool anyInited = kfs[0].initialized && !kfTimedOut;
        
        // ----- 画角点框 -----
        if (anyInited)
>>>>>>> Stashed changes
        {
            for (auto& kf : kfs) kf.initialized = false;
            tvecSmoother.buf.clear();
            consecutiveLost = 0;
            cerr << "\n[WARN] KF timed out after " << MAX_LOST_FRAMES
                 << " consecutive lost frames, resetting filters.\n";
        }

        bool anyInited = kfs[0].initialized && !kfTimedOut;

        // ----- 画角点框 + PnP + 显示（仅在真实检出四角点时执行） -----
        if (anyInited && detected)
        {
            // ---- 绘制箱子边框和角点 ----
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
                // ========== 第一道防线：raw Z 直接检查（平滑前） ==========
                // 必须用 raw tvec 而不是平滑后的值，否则离群值会被平滑器吞掉、
                // z-range 形同虚设，画面数值卡死不动
                double rawZ = tvec.at<double>(2);
                if (rawZ >= 100.0 && rawZ <= 500.0)
                {
                // ---- tvec 平滑 ----
                Vec3d tv(tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2));
                Vec3d tvSmooth = tvecSmoother.push(tv);

                double X    = tvSmooth[0];
                double Y    = tvSmooth[1];
                double Z    = tvSmooth[2];
<<<<<<< Updated upstream
                double dist = sqrt(X*X + Y*Y + Z*Z);

                // 更新全局显示变量
                gX = X; gY = Y; gZ = Z; gDist = dist;
                gHavePose = true;
=======
                double dist = sqrt(X*X + Y*Y + Z*Z);  // 计算距离

                // ========== PnP 结果二重校验 ==========
                bool pnp_valid = true;
                const char* rejectReason = "";

                // 校验1：重投影误差（>5px 视为不可靠）
                {
                    vector<Point2f> reproj;
                    projectPoints(OBJ_PTS, rvec, tvec, newK, zeroDist, reproj);
                    double sumErr = 0;
                    for (int i = 0; i < 4; i++)
                        sumErr += norm(reproj[i] - smoothCorners[i]);
                    double avgErr = sumErr / 4.0;
                    if (avgErr > 5.0) { pnp_valid = false; rejectReason = "reproj"; }
                }

                // 校验2：帧间跳变（相邻帧 Z 变化 >30% 且绝对值 >200mm 丢弃）
                if (pnp_valid && prevValidDist > 0)
                {
                    double jump = fabs(Z - prevValidDist);
                    double relJump = jump / prevValidDist;
                    if (relJump > 0.30 && jump > 200.0)
                    {
                        pnp_valid = false;
                        rejectReason = "frame-jump";
                    }
                }

                if (pnp_valid) prevValidDist = Z;
>>>>>>> Stashed changes

                // ---- 旋转矩阵 / 欧拉角 ----
                Mat R;
                Rodrigues(rvec, R);
                Vec3d eu = euler(R);
                gRoll = eu[0]; gPitch = eu[1]; gYaw = eu[2];

                // ---- 发布 TF 变换（仅校验通过时） ----
                if (pnp_valid)
                {
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = node->get_clock()->now();
                transformStamped.header.frame_id = CAMERA_FRAME_ID;
                transformStamped.child_frame_id = OBJECT_FRAME_ID;

                transformStamped.transform.translation.x = X / 1000.0;
                transformStamped.transform.translation.y = Y / 1000.0;
                transformStamped.transform.translation.z = Z / 1000.0;

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
<<<<<<< Updated upstream
=======
                RCLCPP_INFO(node->get_logger(), "fabush TF: dist=%.1fmm, roll=%.1f, pitch=%.1f, yaw=%.1f",
                            dist, eu[0], eu[1], eu[2]);

                }
                else
                {
                    // 校验失败，不发布TF（可视化仍继续，方便调试）
                    static string lastReason;
                    if (rejectReason != lastReason)
                    {
                        cerr << "\n[REJECT] PnP result discarded: " << rejectReason << "\n";
                        lastReason = rejectReason;
                    }
                }
>>>>>>> Stashed changes

                // ---- 投影坐标轴（轴原点用角点质心，保证永远在框中心） ----
                Point2f axisOrigin = (smoothCorners[0] + smoothCorners[1] +
                                      smoothCorners[2] + smoothCorners[3]) * 0.25f;

                // 投影方向端点（用于提取各轴在图像中的方向）
                Mat tvecSmoothed = (Mat_<double>(3,1) << X, Y, Z);
                vector<Point3f> axisPts = {
                    {0,   0,   0},
                    {100, 0,   0},
                    {0,   100, 0},
                    {0,   0, 100}
                };
                vector<Point2f> axisImg;
                projectPoints(axisPts, rvec, tvecSmoothed, newK, zeroDist, axisImg);

                // 从投影结果提取方向向量，缩放到固定像素长度
                Point2f dX = axisImg[1] - axisImg[0];
                Point2f dY = axisImg[2] - axisImg[0];
                Point2f dZ = axisImg[3] - axisImg[0];
                const float AX = 50.0f;  // 轴长（像素）
                float nx = (float)norm(dX), ny = (float)norm(dY), nz = (float)norm(dZ);
                if (nx > 1e-3f) dX *= AX / nx;
                if (ny > 1e-3f) dY *= AX / ny;
                if (nz > 1e-3f) dZ *= AX / nz;

                // 画坐标轴箭头（从角点质心出发）
                arrowedLine(show, axisOrigin, axisOrigin + dX, Scalar(0,   0,   255), 3, LINE_AA, 0, 0.2);  // X-红
                arrowedLine(show, axisOrigin, axisOrigin + dY, Scalar(0,   255, 0  ), 3, LINE_AA, 0, 0.2);  // Y-绿
                arrowedLine(show, axisOrigin, axisOrigin + dZ, Scalar(255, 0,   0  ), 3, LINE_AA, 0, 0.2);  // Z-蓝

                // 坐标轴标签
                putLabel(show, "X", axisOrigin + dX + Point2f(5, 0),  0.6, Scalar(0,   0,   255));
                putLabel(show, "Y", axisOrigin + dY + Point2f(5, 0),  0.6, Scalar(0,   255, 0  ));
                putLabel(show, "Z", axisOrigin + dZ + Point2f(5, 0),  0.6, Scalar(255, 100, 0  ));

                // ---- 箱子中心十字准星（用角点质心，不用 PnP 投影） ----
                {
                    int cx = (int)axisOrigin.x;
                    int cy = (int)axisOrigin.y;
                    line(show, Point(cx-14, cy),   Point(cx+14, cy),   Scalar(0,255,255), 2, LINE_AA);
                    line(show, Point(cx, cy-14),   Point(cx, cy+14),   Scalar(0,255,255), 2, LINE_AA);
                    circle(show, axisOrigin, 5, Scalar(0,255,255), -1, LINE_AA);
<<<<<<< Updated upstream
                }

                // ---- 控制台输出 ----
                printf("\rDist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  RPY=[%.1f, %.1f, %.1f]deg   ",
                       dist, X, Y, Z, eu[0], eu[1], eu[2]);
                fflush(stdout);
            }  // end if (pnp_ok)
        }  // end if (anyInited && detected)

        // ---- 信息面板：始终显示（只要有历史位姿数据） ----
        if (gHavePose)
        {
            Mat overlay = show.clone();
            rectangle(overlay, Point(10, 10), Point(520, 185), Scalar(0,0,0), FILLED);
            addWeighted(overlay, 0.45, show, 0.55, 0, show);

            int bx = 20, by = 35, dy = 30;
            char buf[256];

            sprintf(buf, "Distance : %.1f mm", gDist);
            putLabel(show, buf, Point(bx, by), 0.78, Scalar(0,255,100), 2);

            sprintf(buf, "X=%.1f  Y=%.1f  Z=%.1f  (mm)", gX, gY, gZ);
            putLabel(show, buf, Point(bx, by+dy), 0.62, Scalar(0,220,255));

            sprintf(buf, "Roll=%.1f  Pitch=%.1f  Yaw=%.1f  (deg)", gRoll, gPitch, gYaw);
            putLabel(show, buf, Point(bx, by+dy*2), 0.62, Scalar(255,200,0));

            string yoloTag = yoloBox.empty() ? "YOLO:LOST" : "YOLO:OK";
            const char* detectTag = detected ? "OK" : "LOST";
            sprintf(buf, "[ DETECT: %s | %s ]", detectTag, yoloTag.c_str());
            putLabel(show, buf, Point(bx, by+dy*3), 0.55,
                     detected ? Scalar(0,255,0) : Scalar(0,140,255));

            sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                   smoothCorners[0].x, smoothCorners[0].y,
                   smoothCorners[1].x, smoothCorners[1].y,
                   smoothCorners[2].x, smoothCorners[2].y,
                   smoothCorners[3].x, smoothCorners[3].y);
            putLabel(show, buf, Point(bx, by+dy*4), 0.52, Scalar(180,180,180));
=======
                }
                
                // ---- 信息面板（左上角半透明背景） ----
                {
                    // 画半透明黑底
                    Mat overlay = show.clone();
                    rectangle(overlay, Point(10, 10), Point(520, 215), Scalar(0,0,0), FILLED);
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

                    // PnP校验状态（仅校验失败时显示）
                    if (!pnp_valid)
                    {
                        string pnpStatus = string("[ PNP REJECT: ") + rejectReason + " ]";
                        putLabel(show, pnpStatus, Point(bx, by + dy*4),
                                0.55, Scalar(0, 100, 255));
                    }

                    // 角点像素坐标
                    int cornerLine = (!pnp_valid) ? 5 : 4;
                    sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                           smoothCorners[0].x, smoothCorners[0].y,
                           smoothCorners[1].x, smoothCorners[1].y,
                           smoothCorners[2].x, smoothCorners[2].y,
                           smoothCorners[3].x, smoothCorners[3].y);
                    putLabel(show, buf, Point(bx, by + dy*cornerLine),
                            0.52, Scalar(180, 180, 180));
                    
                    // 控制台输出
                    printf("\rDist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  "
                          "RPY=[%.1f, %.1f, %.1f]deg   ",
                          dist, X, Y, Z, eu[0], eu[1], eu[2]);
                    fflush(stdout);
                }

            }  // end if (rawZ in range)
            }
>>>>>>> Stashed changes
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
            lastYoloBox = Rect();
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