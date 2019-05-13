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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/aruco.hpp"
#include<mutex>

#include <iostream>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

void * FrameDrawer::getPC()
{
    unique_lock<mutex> lock(mMutex);
    float (*res)[3] = new float[4][3];
    memcpy(res, pc, 3*4*sizeof(pc[0][0]));
    return (void *)res;
}

void * FrameDrawer::getPC_END()
{
    unique_lock<mutex> lock(mMutex);
    float (*res)[3] = new float[4][3];
    memcpy(res, pc_end, 3*4*sizeof(pc_end[0][0]));
    return (void *)res;
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroyq
    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);


  
    vector<int> markerIds;
    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    cv::aruco::detectMarkers(im, markerDictionary, markerCorners, markerIds);

    vector<cv::Vec3d> rvec, tvec;
    cv::aruco::drawDetectedMarkers(im, markerCorners, markerIds, cv::Scalar(0, 255, 0));
   
   /* Estimate axies*/
    // cv::Mat intrinsic;
    // cv::Mat distortion;
    // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.1016f, intrinsic, distortion, rvec, tvec); 
    // for (int index_marker = 0; index_marker < markerIds.size(); index_marker++)
    // {
    // cv::aruco::drawAxis(im, intrinsic, distortion, rvec[index_marker], tvec[index_marker], 0.1);
    // cout << index_marker << endl;
    // }



    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        std::vector<std::vector<cv::Mat>> matching_points_start_marker(4, std::vector<cv::Mat>());
        std::vector<std::vector<cv::Mat>> matching_points_end_marker(4, std::vector<cv::Mat>());
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::Scalar scalar(0,255,255);

                    int vec_idx = -1;
                    for(auto rectagle : markerCorners)
                    {
                        ++vec_idx;
                        int marker_id = markerIds[vec_idx];
                        int idx = 0;
                        for(auto point : rectagle)
                        {
                            float x=pt1.x-point.x;
                            float y=pt1.y-point.y;
                            if(x<0.0)x=-x;
                            if(y<0.0)y=-y;
                            if(x<10 && y<10) {
                                scalar=cv::Scalar(80,80,80);
                                int font = cv::FONT_HERSHEY_SIMPLEX;
                                if (mvCurrentMapPoints[i] != NULL) {
                                    if (mvCurrentMapPoints[i]->GetNormalSize()>0) {
                                        if (marker_id == 25) {
                                            matching_points_start_marker[idx].push_back(mvCurrentMapPoints[i]->GetWorldPos());
                                        }
                                        else if (marker_id == 5) { 
                                            matching_points_end_marker[idx].push_back(mvCurrentMapPoints[i]->GetWorldPos());
                                        }
                                        cv::putText(im,"P("+std::to_string(mvCurrentMapPoints[i]->GetNormaAt(0)) +
                                                       ", "+std::to_string(mvCurrentMapPoints[i]->GetNormaAt(1)) +
                                                       ", "+std::to_string(mvCurrentMapPoints[i]->GetNormaAt(2)) + ")",vCurrentKeys[i].pt, font, 0.5,(255,255,255),1,cv::LINE_AA);
                                        break;
                                    }
                                }
                            }
                            idx++;
                        }
                    }
                    cv::rectangle(im,pt1,pt2,scalar);
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,255),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
        float pc_tmp[4][3];
        int ind = 0;
        for (auto vec : matching_points_start_marker) {
            float x=0,y=0,z=0;
            for(auto point : vec) {
                x += point.at<float>(0);
                y += point.at<float>(1);
                z += point.at<float>(2);
            }
            if (!vec.empty())
            {
                x/=vec.size();
                y/=vec.size();
                z/=vec.size();
            }
            pc_tmp[ind][0] = x;
            pc_tmp[ind][1] = y;
            pc_tmp[ind][2] = z;
            ++ind;
            std::cout<<"("+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+")\n";
        }
        if (!matching_points_start_marker.empty()) {
            unique_lock<mutex> lock(mMutex);
            for (int p1=0; p1<4; ++p1) {
                for (int r1=0; r1<3; ++r1) {
                    pc[p1][r1] = pc_tmp[p1][r1]; 
                }
            }
        }
        //////////////////////  END MARKER
        ind = 0;
        for (auto vec : matching_points_end_marker) {
            float x=0,y=0,z=0;
            for(auto point : vec) {
                x += point.at<float>(0);
                y += point.at<float>(1);
                z += point.at<float>(2);
            }
            if (!vec.empty())
            {
                x/=vec.size();
                y/=vec.size();
                z/=vec.size();
            }
            pc_tmp[ind][0] = x;
            pc_tmp[ind][1] = y;
            pc_tmp[ind][2] = z;
            ++ind;
            std::cout<<"("+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+")\n";
        }
        if (!matching_points_end_marker.empty()) {
            unique_lock<mutex> lock(mMutex);
            for (int p1=0; p1<4; ++p1) {
                for (int r1=0; r1<3; ++r1) {
                    pc_end[p1][r1] = pc_tmp[p1][r1]; 
                }
            }
        }
        std::cout<<std::endl;
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    // std::cout<<"MIRZE: pTracker: mCurrentFrame.mvKeys.size()=" << pTracker->mCurrentFrame.mvKeys.size() << 
    // " mCurrentFrame.mvpMapPoints.size()" << pTracker->mCurrentFrame.mvpMapPoints.size() << std::endl;

    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvCurrentMapPoints=pTracker->mCurrentFrame.mvpMapPoints;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
