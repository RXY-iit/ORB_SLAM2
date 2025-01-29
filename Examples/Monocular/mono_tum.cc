/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<sstream>
#include<string>
#include<vector>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

#include<System.h>
#include "unistd.h"
#include<pangolin/pangolin.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // 输出文件路径以进行调试
    cout << "Vocabulary file: " << argv[1] << endl;
    cout << "Settings file: " << argv[2] << endl;
    cout << "Sequence path: " << argv[3] << endl;

    try {
        cout << "Attempting to open settings file: " << argv[2] << endl;
        
        // 读取整个文件内容
        std::ifstream f(argv[2]);
        if (!f.good()) {
            cerr << "Settings file does not exist: " << argv[2] << endl;
            return -1;
        }
        f.close();
        
        // 使用cv::FileStorage的构造函数方式
        cv::FileStorage fsSettings;
        try {
            cout << "Opening settings file..." << endl;
            fsSettings.open(argv[2], cv::FileStorage::READ);
            cout << "Check if file is opened..." << endl;
            
            if (!fsSettings.isOpened()) {
                cerr << "Failed to parse settings file" << endl;
                return -1;
            }
            
            cout << "Settings parsed successfully" << endl;
            
            // 直接读取参数（不使用嵌套结构）
            double fx = 0, fy = 0, cx = 0, cy = 0;
            bool success = true;
            
            cv::FileNode node = fsSettings["Camera.fx"];
            if (!node.empty() && node.isReal()) {
                fx = (double)node;
                cout << "Read fx: " << fx << endl;
            } else {
                cerr << "Failed to read Camera.fx" << endl;
                success = false;
            }
            
            node = fsSettings["Camera.fy"];
            if (!node.empty() && node.isReal()) {
                fy = (double)node;
                cout << "Read fy: " << fy << endl;
            } else {
                cerr << "Failed to read Camera.fy" << endl;
                success = false;
            }
            
            node = fsSettings["Camera.cx"];
            if (!node.empty() && node.isReal()) {
                cx = (double)node;
                cout << "Read cx: " << cx << endl;
            } else {
                cerr << "Failed to read Camera.cx" << endl;
                success = false;
            }
            
            node = fsSettings["Camera.cy"];
            if (!node.empty() && node.isReal()) {
                cy = (double)node;
                cout << "Read cy: " << cy << endl;
            } else {
                cerr << "Failed to read Camera.cy" << endl;
                success = false;
            }
            
            if (!success) {
                cerr << "Failed to read camera parameters" << endl;
                fsSettings.release();
                return -1;
            }
            
            // 验证参数值
            if(fx <= 0 || fy <= 0 || cx <= 0 || cy <= 0) {
                cerr << "Invalid camera parameters" << endl;
                cerr << "fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << endl;
                fsSettings.release();
                return -1;
            }
            
            cout << "All camera parameters read successfully:" << endl;
            cout << "fx = " << fx << endl;
            cout << "fy = " << fy << endl;
            cout << "cx = " << cx << endl;
            cout << "cy = " << cy << endl;
            
            // 读取其他参数
            double fps = 30.0;
            int rgb = 1;
            
            node = fsSettings["Camera.fps"];
            if (!node.empty() && node.isReal()) {
                fps = (double)node;
                cout << "Read fps: " << fps << endl;
            }
            
            node = fsSettings["Camera.RGB"];
            if (!node.empty() && node.isInt()) {
                rgb = (int)node;
                cout << "Read RGB: " << rgb << endl;
            }
            
            cout << "Additional parameters:" << endl;
            cout << "fps = " << fps << endl;
            cout << "RGB = " << (rgb ? "true" : "false") << endl;
            
            fsSettings.release();
            
            // 读取ORB参数
            cv::FileNode orbNode = fsSettings["ORBextractor"];
            if (!orbNode.empty()) {
                int nFeatures = (int)orbNode["nFeatures"];
                float scaleFactor = (float)orbNode["scaleFactor"];
                int nLevels = (int)orbNode["nLevels"];
                
                cout << "ORB parameters:" << endl;
                cout << "nFeatures = " << nFeatures << endl;
                cout << "scaleFactor = " << scaleFactor << endl;
                cout << "nLevels = " << nLevels << endl;
            }
            
            // Retrieve paths to images
            vector<string> vstrImageFilenames;
            vector<double> vTimestamps;
            string strFile = string(argv[3])+"/rgb.txt";
            LoadImages(strFile, vstrImageFilenames, vTimestamps);

            int nImages = vstrImageFilenames.size();

            // Create SLAM system. It initializes all system threads and gets ready to process frames.
            ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

            // Vector for tracking time statistics
            vector<float> vTimesTrack;
            vTimesTrack.resize(nImages);

            cout << endl << "-------" << endl;
            cout << "Start processing sequence ..." << endl;
            cout << "Images in the sequence: " << nImages << endl << endl;

            // Main loop
            cv::Mat im;
            for(int ni=0; ni<nImages; ni++)
            {
                // Read image from file
                im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
                double tframe = vTimestamps[ni];

                if(im.empty())
                {
                    cerr << endl << "Failed to load image at: "
                         << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
                    return 1;
                }

        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif

                // Pass the image to the SLAM system
                SLAM.TrackMonocular(im,tframe);

        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

                double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

                vTimesTrack[ni]=ttrack;

                // Wait to load the next frame
                double T=0;
                if(ni<nImages-1)
                    T = vTimestamps[ni+1]-tframe;
                else if(ni>0)
                    T = tframe-vTimestamps[ni-1];

                if(ttrack<T)
                    usleep((T-ttrack)*1e6);
            }
            // // 检查并创建保存目录
            // string save_dir = "/home/ruan-x/Documents/save_data/";
            // if(system(("mkdir -p " + save_dir).c_str()) != 0) {
            //     cerr << "Failed to create directory: " << save_dir << endl;
            // }

       

            // 先保存文本格式的数据
            try {
                vector<ORB_SLAM2::MapPoint*> vpMPs = SLAM.GetMap()->GetAllMapPoints();
                cout << "Number of MapPoints: " << vpMPs.size() << endl;
                
                SLAM.SaveMapPoints("/home/ruan-x/Documents/save_data/MapPoints.txt");
                cout << "MapPoints.txt output finished! " << endl;
            } catch (const exception& e) {
                cerr << "Error saving map points: " << e.what() << endl;
            }
            
            // 保存轨迹
            SLAM.SaveMonocularTrajectory("/home/ruan-x/Documents/save_data/trajectory.txt");
            cout << "trajectory.txt output finished! " << endl;
            
            // 保存关键帧轨迹
            SLAM.SaveKeyFrameTrajectoryTUM("/home/ruan-x/Documents/save_data/KeyFrameTrajectory.txt");

            // 先关闭SLAM系统和所有窗口
            SLAM.Shutdown();

            // 等待一会儿确保所有窗口都关闭
            usleep(1000000);  // 等待1秒

            // 尝试保存 PLY 文件
            try {
                // 获取地图点和关键帧
                vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
                vector<ORB_SLAM2::KeyFrame*> keyFrames = SLAM.GetMap()->GetAllKeyFrames();
                
                // 创建保存目录
                string save_dir = "/home/ruan-x/Documents/save_data/";
                system(("mkdir -p " + save_dir).c_str());

                // 保存地图点到 PLY 文件
                string map_points_file = save_dir + "map_points.ply";
                ofstream f_points(map_points_file);
                if (f_points.is_open()) {
                    // 计算有效的地图点数量
                    int validPoints = 0;
                    for(auto mp : mapPoints) {
                        if(!mp->isBad()) validPoints++;
                    }

                    // 写入 PLY 文件头
                    f_points << "ply" << endl;
                    f_points << "format ascii 1.0" << endl;
                    f_points << "element vertex " << validPoints << endl;  // 只计算有效点
                    f_points << "property float x" << endl;
                    f_points << "property float y" << endl;
                    f_points << "property float z" << endl;
                    f_points << "property uchar red" << endl;
                    f_points << "property uchar green" << endl;
                    f_points << "property uchar blue" << endl;
                    f_points << "end_header" << endl;

                    // 写入地图点
                    for(auto mp : mapPoints) {
                        if(mp->isBad()) continue;
                        cv::Mat pos = mp->GetWorldPos();
                        f_points << fixed << setprecision(5)  // 设置输出精度
                                << pos.at<float>(0) << " " 
                                << pos.at<float>(1) << " " 
                                << pos.at<float>(2) << " "
                                << "255 0 0" << endl;  // 红色点
                    }
                    f_points.close();
                    cout << "Saved " << validPoints << " map points to: " << map_points_file << endl;
                }

                // 保存关键帧到 PLY 文件
                string keyframes_file = save_dir + "keyframes.ply";
                ofstream f_frames(keyframes_file);
                if (f_frames.is_open()) {
                    // 计算有效的关键帧数量
                    int validFrames = 0;
                    for(auto kf : keyFrames) {
                        if(!kf->isBad()) validFrames++;
                    }

                    // 写入 PLY 文件头
                    f_frames << "ply" << endl;
                    f_frames << "format ascii 1.0" << endl;
                    f_frames << "element vertex " << validFrames * 4 << endl;  // 每个关键帧用4个点表示
                    f_frames << "property float x" << endl;
                    f_frames << "property float y" << endl;
                    f_frames << "property float z" << endl;
                    f_frames << "property uchar red" << endl;
                    f_frames << "property uchar green" << endl;
                    f_frames << "property uchar blue" << endl;
                    f_frames << "element edge " << validFrames * 4 << endl;  // 连接点形成相机框
                    f_frames << "property int vertex1" << endl;
                    f_frames << "property int vertex2" << endl;
                    f_frames << "end_header" << endl;

                    int validIdx = 0;  // 用于跟踪有效关键帧的索引
                    for(auto kf : keyFrames) {
                        if(kf->isBad()) continue;
                        
                        cv::Mat Twc = kf->GetPoseInverse();
                        float sz = 0.1;  // 相机框大小
                        
                        // 相机框的四个角点
                        cv::Mat p1 = (cv::Mat_<float>(4,1) << -sz, -sz/2, 0, 1);
                        cv::Mat p2 = (cv::Mat_<float>(4,1) << sz, -sz/2, 0, 1);
                        cv::Mat p3 = (cv::Mat_<float>(4,1) << sz, sz/2, 0, 1);
                        cv::Mat p4 = (cv::Mat_<float>(4,1) << -sz, sz/2, 0, 1);
                        
                        // 转换到世界坐标系
                        p1 = Twc * p1;
                        p2 = Twc * p2;
                        p3 = Twc * p3;
                        p4 = Twc * p4;
                        
                        // 写入四个角点
                        f_frames << fixed << setprecision(5)
                                << p1.at<float>(0) << " " << p1.at<float>(1) << " " << p1.at<float>(2) << " 0 255 0" << endl
                                << p2.at<float>(0) << " " << p2.at<float>(1) << " " << p2.at<float>(2) << " 0 255 0" << endl
                                << p3.at<float>(0) << " " << p3.at<float>(1) << " " << p3.at<float>(2) << " 0 255 0" << endl
                                << p4.at<float>(0) << " " << p4.at<float>(1) << " " << p4.at<float>(2) << " 0 255 0" << endl;
                        
                        // 写入边
                        int base = validIdx * 4;
                        f_frames << base << " " << base+1 << endl
                                << base+1 << " " << base+2 << endl
                                << base+2 << " " << base+3 << endl
                                << base+3 << " " << base << endl;
                        
                        validIdx++;
                    }
                    f_frames.close();
                    cout << "Saved " << validFrames << " keyframes to: " << keyframes_file << endl;
                }
            } catch (const std::exception& e) {
                cerr << "Error saving PLY files: " << e.what() << endl;
            }

            // Tracking time statistics
            sort(vTimesTrack.begin(),vTimesTrack.end());
            float totaltime = 0;
            for(int ni=0; ni<nImages; ni++)
            {
                totaltime+=vTimesTrack[ni];
            }
            cout << "-------" << endl << endl;
            cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
            cout << "mean tracking time: " << totaltime/nImages << endl;

            return 0;
        } catch(const cv::Exception& e) {
            cerr << "OpenCV exception: " << e.what() << endl;
            cerr << "Error code: " << e.code << endl;
            cerr << "Error line: " << e.line << endl;
            cerr << "Error file: " << e.file << endl;
            return -1;
        } catch(const std::exception& e) {
            cerr << "Standard exception: " << e.what() << endl;
            return -1;
        }
    } catch(const cv::Exception& e) {
        cerr << "OpenCV exception: " << e.what() << endl;
        cerr << "Error code: " << e.code << endl;
        cerr << "Error line: " << e.line << endl;
        cerr << "Error file: " << e.file << endl;
        return -1;
    } catch(const std::exception& e) {
        cerr << "Standard exception: " << e.what() << endl;
        return -1;
    }
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
