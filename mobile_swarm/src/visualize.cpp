#include "../include/visualize.h"

void DrawGrid(int gridlines = 15, float linewidth = 2.0)
{
    if (gridlines <= 0 || gridlines > 100) {
        gridlines = 15;
    }
    if (linewidth <= 0) {
        linewidth = 2.0;
    }

    glPointSize(15.0);
    glBegin( GL_POINTS );       
    glColor3f(1.0,0.647,0.0);
    glVertex3f(0, 0, 0);
    glEnd();
    glLineWidth(linewidth);
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 0.0);
    for (int i = -gridlines; i <= gridlines; ++i) {
        glVertex3d(i, -gridlines, 0);
        glVertex3d(i, gridlines, 0);
        glVertex3d(-gridlines, i, 0);
        glVertex3d(gridlines, i, 0);
    }
    glEnd();

    /* */
    // glLineWidth(5.0);
    // glBegin(GL_LINES);
    // glColor3f(1.0, 0.0, 0.0);
    // glVertex3d(0, 0, 0);
    // glVertex3d(0, 3, 0);
    // glVertex3d(0, 3, 0);
    // glVertex3d(3, 3, 0);
    // glVertex3d(3, 3, 0);
    // glVertex3d(3, 0, 0);
    // glVertex3d(3, 0, 0);
    // glVertex3d(0, 0, 0);
    // glEnd();
    /* */
}

void DrawBot(const vector<float>& botPos)
{
    const float _botHalfWidth = 0.1, _botHalfHeight = 0.15;
    if (botPos.empty()) {
        return ;
    }

    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glVertex3f(botPos[0] + _botHalfWidth, botPos[1] + _botHalfHeight, 0);
    glVertex3f(botPos[0] - _botHalfWidth, botPos[1] - _botHalfHeight, 0);
    glEnd();
}

void DrawLaserScan(const vector<vector<float>>& laserData, const vector<float>& botPos)
{
    if (laserData.empty() || botPos.empty()) {
        return ;
    }

    glPointSize(5.0);
    glBegin( GL_POINTS ); 
    glColor3f(1.0,0.0,0.0);
    for (int i = 0; i < static_cast<int>(laserData.size()); ++i) {
        glVertex3f(laserData[i][0] + botPos[0], laserData[i][1] + botPos[1], 0);
    }
    glEnd(); 
}

void DrawObstacle(const vector<float>& obsCoor, const vector<float>& botPos)
{
    if (obsCoor.empty() || botPos.empty()) {
        return ;
    }
    glPointSize(15.0);
    glBegin( GL_POINTS );       
    glColor3f(1.0,0.647,0.0);
    glVertex3f(obsCoor[0] + botPos[0], obsCoor[1] + botPos[1], 0);
    glEnd();
}

void DrawPath(const vector<vector<float>>& botPath, int idx)
{
    // 配色方案：https://colordrop.io
    const static vector<vector<float>> _pathColorLib = {{0.894f, 0.090f, 0.286f}, {1.f, 0.635f, 0.f},
                    {0.0f, 0.627f, 0.243f}, {0.f, 0.529f, 0.796f}};
    if (botPath.empty() || botPath.size() < 1) {
        return ;
    }

    glLineWidth(4);
    glBegin(GL_LINES);
    glColor3f(_pathColorLib[idx][0], _pathColorLib[idx][1], _pathColorLib[idx][2]);

    vector<float> _prePoint = botPath[0];
    for( int i = 1; i < botPath.size(); ++i){
        glVertex3d(_prePoint[0], _prePoint[1], 0);
        glVertex3d(botPath[i][0], botPath[i][1], 0);
        _prePoint = botPath[i];
    }
    glEnd();
}

void DrawTargetPos(double pos[])
{
    glPointSize(25.0);
    glBegin( GL_POINTS );       
    glColor3f(1.0,0.0,0.0);
    glVertex3f(pos[0], pos[1], 0);
    glEnd();
}

void UploadImg(cv::Mat& img, pangolin::View& upview, pangolin::GlTexture& uptext, cv::Size upsize)
{
    if (!img.empty()) {
        cv::resize(img, img, upsize);
    }
    if (img.size().width == upsize.width && 
        img.size().height == upsize.height) {
        upview.Activate();
        uptext.Upload(img.data, GL_BGR, GL_UNSIGNED_BYTE);
        glColor3f(1.0,1.0,1.0);
        uptext.RenderToViewportFlipY();
    }
}   

// 将坐标表示的单精度浮点数转换为保留小数点后三位的字符串
string botCoor_f2str(const vector<float>& botPos)
{
    string _contvert_to_str;
    if (botPos.empty()) {
        return "";
    }

    int _pre3nums_0 = abs((int)(botPos[0] * 1000) / 1000);
    int _after3nums_0 = abs((int)(botPos[0] * 1000) % 1000);
    int _pre3nums_1 = abs((int)(botPos[1] * 1000) / 1000);
    int _after3nums_1 = abs((int)(botPos[1] * 1000) % 1000);
    int _pre3nums_2 = abs((int)(botPos[2] * 1000) / 1000);
    int _after3nums_2 = abs((int)(botPos[2] * 1000) % 1000);

    _contvert_to_str = (string)"[" + (botPos[0] < 0 ? "-" : "") + to_string(_pre3nums_0) + "." + 
            (_after3nums_0 < 100 ? "0" : "") + (_after3nums_0 < 10 ? "0" : "") + to_string(_after3nums_0) + "," +
            (botPos[1] < 0 ? "-" : "") + to_string(_pre3nums_1) + "." + (_after3nums_1 < 100 ? "0" : "") +
            (_after3nums_1 < 10 ? "0" : "") + to_string(_after3nums_1) + "," +
            (botPos[2] < 0 ? "-" : "") + to_string(_pre3nums_2) + "." + (_after3nums_2 < 100 ? "0" : "") +
            (_after3nums_2 < 10 ? "0" : "") + to_string(_after3nums_2) + "]";
    return _contvert_to_str;
}

