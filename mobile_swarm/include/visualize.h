#pragma once
#ifndef __VISUALIZE_H
#define __VISUALIZE_H

#include "common_include.h"

void DrawGrid(int gridlines, float linewidth);
void DrawBot(const vector<float>& botPos);
void DrawLaserScan(const vector<vector<float>>& laserData, const vector<float>& botPos);
void DrawObstacle(const vector<float>& obsCoor, const vector<float>& botPos);
void DrawPath(const list<vector<float>>& botPath, int idx);
void DrawTargetPos(double pos[]);
void UploadImg(cv::Mat& img, pangolin::View& upview, pangolin::GlTexture& uptext, cv::Size upsize);
string botCoor_f2str(const vector<float>& botPos);


#endif

