#include <chrono>
#include <ctime>

#include "../include/common_include.h"
#include "../include/visualize.h"
#include "../include/agent.h"
#include "../include/view_handler.h"

#define SCAN_SHOW_FPS 30

using namespace std;

const constexpr int FRAME_DELAY_TIME = static_cast<int>(1000 / SCAN_SHOW_FPS);
const constexpr int FEATURE_IMAGE_WIDTH = 640;
const constexpr int FEATURE_IMAGE_HEIGHT = 480;

// 函数声明
void MainShow(ros::NodeHandle* n);
void RosSpin(void);
void tarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

// 全局变量定义
vector<Agent*> agents;
bool shouldQuit = false;
bool showTargetPos = false;
double tar_pos[3];

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Mobile_Swarm_Server");
    ros::NodeHandle n;

    Agent vins_1("vins_1", &n);
    Agent vins_2("vins_2", &n);
    Agent vins_3("vins_3", &n);
    agents.emplace_back(&vins_1);
    agents.emplace_back(&vins_2);
    agents.emplace_back(&vins_3);
             
    std::thread MainShow_thread(MainShow, &n);
    std::thread RosSpin_thread(RosSpin);
    MainShow_thread.join();
    RosSpin_thread.join();

    // ros::spin();
    return 0;
}

void RosSpin()
{
    while (!shouldQuit) {
        ros::spinOnce();
    }
    shouldQuit = true;
    return ;
}

void MainShow(ros::NodeHandle* n)
{
    if (agents.size() != 3) {
        return ;
    }

    ros::Subscriber sub_tar = n->subscribe("target_pos", 1000, tarCallback);

    vector<string> _viewPort_Str = {"mainView", "agent_0", "agent_1", "agent_2"};
    int _curViewPort = 0;

    pangolin::CreateWindowAndBind("Mobile Swarm",1080,720);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1080,720,420,420,320,320,0.2,100),
            pangolin::ModelViewLookAt(-1,1.5,3,-1,1.5,0, pangolin::AxisY)
    );
    
    pangolin::myHandler handler(s_cam, n);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1080.0f/720.0f)
            .SetHandler(&handler);

    pangolin::CreatePanel("pannel").SetBounds(0.28, 1.0, 0.0, 0.25);    // 创建
    pangolin::Var<int> _viewPort("pannel.viewPort", 0, 0, 3);           // 设置slider
    pangolin::Var<string> _viewPortStr("pannel.Cur:");                  // 设置一个按钮
    pangolin::Var<string> _curCoordinate("pannel.T:");                  // 设置一个按钮
    pangolin::Var<bool> _showGrid("pannel.showGrid", true, true);       // 设置一个按钮
    pangolin::Var<bool> _showPath("pannel.showPath", false, true);      // 设置一个按钮
    pangolin::Var<bool> _showCameraView("pannel.showCameraView", false, true);  // 设置一个按钮
    pangolin::Var<bool> _showLaserPoint("pannel.showLaserPoint", false, true);  // 设置一个按钮
    pangolin::Var<bool> _showObstacle("pannel.showObstacle", false, true);      // 设置一个按钮
    pangolin::Var<bool> _resetBotton("pannel.resetCameraView", false, false);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);


    pangolin::View& _showFeatureImg_1 = pangolin::Display("feature_1")
        .SetBounds(0, 0.5, 0, 0.25, 1080.0f/720.0f)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::GlTexture _imageTexture_1(FEATURE_IMAGE_WIDTH,FEATURE_IMAGE_HEIGHT,
        GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    pangolin::View& _showFeatureImg_2 = pangolin::Display("feature_2")
        .SetBounds(0, 0.5, 0.25, 0.50, 1080.0f/720.0f)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::GlTexture _imageTexture_2(FEATURE_IMAGE_WIDTH,FEATURE_IMAGE_HEIGHT,
        GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    pangolin::View& _showFeatureImg_3 = pangolin::Display("feature_3")
        .SetBounds(0, 0.5, 0.50, 0.75, 1080.0f/720.0f)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::GlTexture _imageTexture_3(FEATURE_IMAGE_WIDTH,FEATURE_IMAGE_HEIGHT,
        GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);

    while(!pangolin::ShouldQuit() && !shouldQuit) {  

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        if (pangolin::Pushed(_resetBotton)) {
            showTargetPos = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1080,720,420,420,320,320,0.2,100));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-1,1.5,3,-1,1.5,0, pangolin::AxisY));
        }

        _curViewPort = _viewPort.Get();
        _viewPortStr = _viewPort_Str[_curViewPort];
        if (_curViewPort != 0) {
            _curCoordinate = botCoor_f2str((*agents[_curViewPort - 1]).getBotCoor());  
        } else {
            _curCoordinate = "[]";
        }

        if (_curViewPort == 0) {
            DrawBot((*agents[0]).getBotCoor());
            DrawBot((*agents[1]).getBotCoor());
            DrawBot((*agents[2]).getBotCoor());
        } else {
            DrawBot((*agents[_curViewPort - 1]).getBotCoor());
        }
        
        if (_showGrid == true) {
            DrawGrid(15, 2.0);
        }

        if (_showLaserPoint == true) {
            if (_curViewPort == 0) {
                (*agents[0]).startSubLaser();
                (*agents[1]).startSubLaser();
                (*agents[2]).startSubLaser();

                DrawLaserScan((*agents[0]).getLaserData(), (*agents[0]).getBotCoor());
                DrawLaserScan((*agents[1]).getLaserData(), (*agents[1]).getBotCoor());
                DrawLaserScan((*agents[2]).getLaserData(), (*agents[2]).getBotCoor());
            } else {
                for (int i = 0; i < 3; ++i) {
                    if ( i == _curViewPort - 1) {
                        (*agents[i]).startSubLaser();
                    } else {
                        (*agents[i]).stopSubImg();
                    }
                }
                DrawLaserScan((*agents[_curViewPort - 1]).getLaserData(), 
                    (*agents[_curViewPort - 1]).getBotCoor());
            }
        } else {
            (*agents[0]).stopSubLaser();
            (*agents[1]).stopSubLaser();
            (*agents[2]).stopSubLaser();
        }

        if (_showObstacle == true) {
            if (_curViewPort == 0) {
                DrawObstacle((*agents[0]).getObsCoor(), (*agents[0]).getBotCoor());
                DrawObstacle((*agents[1]).getObsCoor(), (*agents[1]).getBotCoor());
                DrawObstacle((*agents[2]).getObsCoor(), (*agents[2]).getBotCoor());
            } else {
                DrawObstacle((*agents[_curViewPort - 1]).getObsCoor(), 
                    (*agents[_curViewPort - 1]).getBotCoor());
            }
        }

        if (_showPath == true) {
            if (_curViewPort == 0) {
                DrawPath((*agents[0]).getBotPath(), 0);
                DrawPath((*agents[1]).getBotPath(), 1);
                DrawPath((*agents[2]).getBotPath(), 2);
            } else {
                DrawPath((*agents[_curViewPort - 1]).getBotPath(), _curViewPort - 1);
            }
        }

        if (_showCameraView) {
            if (_curViewPort == 0) {
                (*agents[0]).startSubImg();
                (*agents[1]).startSubImg();
                (*agents[2]).startSubImg();
                cv::Mat _fImgTmp_1 = (*agents[0]).getFeatureImg();
                cv::Mat _fImgTmp_2 = (*agents[1]).getFeatureImg();
                cv::Mat _fImgTmp_3 = (*agents[2]).getFeatureImg();

                if (_fImgTmp_1.size().width == FEATURE_IMAGE_WIDTH && 
                    _fImgTmp_1.size().height == FEATURE_IMAGE_HEIGHT) {
                    _showFeatureImg_1.Activate();
                    _imageTexture_1.Upload(_fImgTmp_1.data, GL_BGR, GL_UNSIGNED_BYTE);
                    glColor3f(1.0,1.0,1.0);
                    _imageTexture_1.RenderToViewportFlipY();
                }

                if (_fImgTmp_2.size().width == FEATURE_IMAGE_WIDTH && 
                    _fImgTmp_2.size().height == FEATURE_IMAGE_HEIGHT) {
                    _showFeatureImg_2.Activate();
                    _imageTexture_2.Upload(_fImgTmp_2.data, GL_BGR, GL_UNSIGNED_BYTE);
                    glColor3f(1.0,1.0,1.0);
                    _imageTexture_2.RenderToViewportFlipY();
                }

                if (_fImgTmp_3.size().width == FEATURE_IMAGE_WIDTH && 
                    _fImgTmp_3.size().height == FEATURE_IMAGE_HEIGHT) {
                    _showFeatureImg_3.Activate();
                    _imageTexture_3.Upload(_fImgTmp_3.data, GL_BGR, GL_UNSIGNED_BYTE);
                    glColor3f(1.0,1.0,1.0);
                    _imageTexture_3.RenderToViewportFlipY();
                }

                _showFeatureImg_1.show = true;
                _showFeatureImg_2.show = true;
                _showFeatureImg_3.show = true;
            } else {
                for (int i = 0; i < 3; ++i) {
                    if (i == _curViewPort - 1) {
                        (*agents[i]).startSubImg();
                    } else {
                        (*agents[i]).stopSubImg();
                    }
                }
                cv::Mat _fImgTmp = (*agents[_curViewPort - 1]).getFeatureImg();

                if (_fImgTmp.size().width == FEATURE_IMAGE_WIDTH && 
                    _fImgTmp.size().height == FEATURE_IMAGE_HEIGHT) {
                    _showFeatureImg_1.Activate();
                    _imageTexture_1.Upload(_fImgTmp.data, GL_BGR, GL_UNSIGNED_BYTE);
                    glColor3f(1.0,1.0,1.0);
                    _imageTexture_1.RenderToViewportFlipY();
                }

                _showFeatureImg_1.show = true;
                _showFeatureImg_2.show = false;
                _showFeatureImg_3.show = false;
            }
        } else {
            (*agents[0]).stopSubImg();
            (*agents[1]).stopSubImg();
            (*agents[2]).stopSubImg();
            _showFeatureImg_1.show = false;
            _showFeatureImg_2.show = false;
            _showFeatureImg_3.show = false;
        }

        if (showTargetPos) {
            DrawTargetPos(tar_pos);
        }

        pangolin::FinishFrame();
        usleep(FRAME_DELAY_TIME);
    }

    shouldQuit = true;
    pangolin::DestroyWindow("Mobile Swarm");
    return ;
}

void tarCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    showTargetPos = true;
    tar_pos[0] = msg->data[0];
    tar_pos[1] = msg->data[1];
}

