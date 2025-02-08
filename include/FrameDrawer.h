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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <chrono>

namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();
    
    // Save tracking info to CSV and image
    void SaveTrackingInfo();
    
    // Check if we need to save the current frame
    bool NeedSaveFrame() {
        unique_lock<mutex> lock(mMutex);
        bool needSave = mbNeedSaveFrame;
        mbNeedSaveFrame = false;  // Reset flag after checking
        return needSave;
    }
    
    // Get current simulation time
    double GetCurrentTime() {
        unique_lock<mutex> lock(mMutex);
        auto current_time = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - mStartTime).count() / 1000.0;
    }

protected:
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbVO, mvbMap;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    bool mbNeedSaveFrame;  // Flag to indicate if we need to save the frame

    Map* mpMap;

    std::mutex mMutex;
    
private:
    // CSV保存相关
    void InitializeCSV();
    string mSaveDir;
    string mKeyFramePicDir;  // 图片保存目录
    std::chrono::steady_clock::time_point mStartTime;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
