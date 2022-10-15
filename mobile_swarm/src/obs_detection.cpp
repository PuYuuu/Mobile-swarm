#include <vector>
#include <algorithm>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"

using namespace std;
#define inf std::numeric_limits<double>::infinity()
#define SCAN_SHOW_FPS 50
#define SUPPORT_PANGOLIN 1

#if SUPPORT_PANGOLIN
#include <thread>
#include <pangolin/pangolin.h>
#endif

struct laserPoint {
    float range;
    float theta;
};

// 函数声明
void laserShow(void);
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void midianFilter(const vector<laserPoint>& _scanOri, vector<laserPoint>& _scanFil, int _windowSize);

// 不同颜色区别不同类别的激光点
// 分别表示猩红、中蓝、纯绿、深天蓝、纯黄、灰色、紫色、道奇蓝、橄榄、珊瑚色、纯黑、粉红
// 详情参见：https://www.sioe.cn/yingyong/yanse-rgb-16/
const vector<vector<float>> colorLib = {{0.863,0.078,0.235}, {0.000,0.000,0.804}, {0.000,0.502,0.000}
            , {0.000,0.749,1.000}, {1.000,1.000,0.000}, {0.502,0.502,0.502}, {0.502,0.000,0.502}
            , {0.118,0.565,1.000}, {0.502,0.502,0.000}, {0.957,0.502,0.502}, {0.000,0.000,0.000}
            , {1.000,0.714,0.757}};

// 全局变量定义
ros::Publisher obs_coor_pub;
geometry_msgs::Point32 obs;
float thresholdGain = 0.25;
int slideWindowSize = 5;

vector<laserPoint> laserData_Origin;
vector<laserPoint> laserData_Filter;
vector<int> laserData_Cluste;


int main(int argc, char** argv)
{
    ros::init(argc,argv,"obs_detectionServer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, laserScancallback);
    obs_coor_pub = n.advertise<geometry_msgs::Point32>("obs_coor",1000);
    ROS_INFO_STREAM("Node \"obs_detection\" initial successful !");

#if SUPPORT_PANGOLIN
    std::thread laserShow_thread;               // 新建一个线程
    laserShow_thread = std::thread(laserShow);  // 然后将函数laserShow加载到线程中
    laserShow_thread.detach();      // 分离主线程和子线程的关联           
#endif
    
    ros::spin();
    return 0;
}


void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 激光雷达扫描一次的激光点数（(最大角度-最小角度)/单位角度 = 激光点的个数）
	// int _scanSize = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);
    static const int _scanSize = 360;           // 思岚A1雷达一圈360个点
    vector<vector<float>> _clusterVarietyCenter(0, vector<float>(2,0));
    vector<float> _varietyTmp(2, 0);
    int _curVarietyIndex = 0;
    int _curVarietyNumber = 1;
    float _curPonitAngle = scan->angle_min;
    float _angle_increment = scan->angle_increment;
    
    laserData_Origin.clear();
    laserData_Filter.clear();
    laserData_Cluste.clear();

    for (int i = 0; i < _scanSize; ++i) {
        if (scan->ranges[i] != inf) {
            // 舍弃无效点           
            laserPoint _tmp = {scan->ranges[i], _curPonitAngle};
            laserData_Origin.emplace_back(_tmp);
            laserData_Filter.emplace_back(_tmp);    // 初始化滤波vector
        }
        _curPonitAngle += _angle_increment;
    }

    midianFilter(laserData_Origin, laserData_Filter, slideWindowSize);

    int _dataFilter_Size = static_cast<int>(laserData_Filter.size());
    laserData_Cluste.resize(_dataFilter_Size);
    laserData_Cluste[0] = _curVarietyIndex;
    _varietyTmp[0] = laserData_Filter[0].range * cos(laserData_Filter[0].theta);
    _varietyTmp[1] = laserData_Filter[0].range * sin(laserData_Filter[0].theta);
    _clusterVarietyCenter.emplace_back(_varietyTmp);

    for (int i = 1; i <= _dataFilter_Size; ++i) {
        // 0.01745 = sin(1 * 3.14 / 180.0)
        float _threshold = thresholdGain * laserData_Filter[i].range * 0.01745;
        float _scanIndex_x = laserData_Filter[i].range * cos(laserData_Filter[i].theta);
        float _scanIndex_y = laserData_Filter[i].range * sin(laserData_Filter[i].theta);

        if (i < _dataFilter_Size && 
            abs(laserData_Filter[i].range - laserData_Filter[i - 1].range) < _threshold) {
            // 同一类点
            laserData_Cluste[i] = _curVarietyIndex;
            _clusterVarietyCenter[_curVarietyIndex][0] += _scanIndex_x;
            _clusterVarietyCenter[_curVarietyIndex][1] += _scanIndex_y;
            _curVarietyNumber++;
        } else {
            // 不同类点
            _clusterVarietyCenter[_curVarietyIndex][0] /= _curVarietyNumber;
            _clusterVarietyCenter[_curVarietyIndex][1] /= _curVarietyNumber;
            _curVarietyNumber = 1;

            if (i < _dataFilter_Size) {
                laserData_Cluste[i] = ++_curVarietyIndex;
                _varietyTmp[0] = _scanIndex_x;
                _varietyTmp[1] = _scanIndex_y;
                _clusterVarietyCenter.emplace_back(_varietyTmp);
            }
        }
    }

    // 将聚类后的点云团按几何中心到原点距离从小到大排序
    // to do : 如果某一类中仅包含一两个点则取次小的
    sort(_clusterVarietyCenter.begin(), _clusterVarietyCenter.end(), 
        [](const vector<float>&a, const vector<float>&b) {
            return a[0] * a[0] + a[1] * a[1] < b[0] * b[0] + b[1] * b[1];});
    obs.x = _clusterVarietyCenter[0][0];
    obs.y = _clusterVarietyCenter[0][1];
    obs.z = 0;
    obs_coor_pub.publish(obs);
}

void midianFilter(const vector<laserPoint>& _scanOri, vector<laserPoint>& _scanFil, int _windowSize)
{
    int _scanOri_Size = static_cast<int>(_scanOri.size());
    // _scanFil = _scanOri;
    if (_scanOri_Size <= 1) {
        return ;
    }
    // 为处理滑窗的边界问题，对称扩展边界信号
    vector<laserPoint> _scanExtension(_scanOri_Size + _windowSize - (_windowSize % 2 != 0));
    for (int i = 0; i < _windowSize / 2; ++i) {
        _scanExtension[i] = _scanOri[_windowSize / 2 - 1 - i];
        _scanExtension[_scanOri_Size + _windowSize / 2 + i] = _scanOri[_scanOri_Size - 1 - i];
    }
    for (int i = 0; i < _scanOri_Size; ++i) {
        _scanExtension[i + (_windowSize - 1) / 2] = _scanOri[i];
    }

    // 使用中值滤波去除噪点，滑窗大小为_windowSize
    for (int i = 0; i < _scanOri_Size ; ++i) {
        vector<float> _window(_windowSize);
        for (int j = 0; j < _windowSize; ++j) {
            _window[j] = _scanExtension[i + j].range;
        }

        for (int j = 0; j < _windowSize / 2 + 1; ++j) {
            for (int k = j + 1; k < _windowSize; ++k) {
                if (_window[k] < _window[j]) {
                    float temp = _window[j];
                    _window[j] = _window[k];
                    _window[k] = temp;
                }
            }
        }
        if (_windowSize % 2 != 0) {
            _scanFil[i].range = _window[_windowSize / 2];
        } else {
            _scanFil[i].range = (_window[_windowSize / 2] + _window[_windowSize / 2 - 1]) / 2.0;
        }
    }
}

#if SUPPORT_PANGOLIN
void drawPoint(const laserPoint& _point)
{
    float _tmpRange = _point.range;
    float _tmpTheta = _point.theta;
    float _tmpX = _tmpRange * cos(_tmpTheta);
    float _tmpY = _tmpRange * sin(_tmpTheta);
    glVertex3f(_tmpX, _tmpY, 0);
}

void laserShow()
{
    static int _showState = 0;
    vector<laserPoint> _laserData_Show;
    pangolin::CreateWindowAndBind("LaserScan",1080,720);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1080,720,420,420,320,320,0.2,100),
            pangolin::ModelViewLookAt(-10,0,12, -10,0,0, pangolin::AxisY)
    );
    
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1080.0f/720.0f)
            .SetHandler(&handler);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));// 创建
    pangolin::Var<bool> a_button("ui.laserData_Origin", false, false);  // 设置一个按钮
    pangolin::Var<bool> b_button("ui.laserData_Filter", false, false);  // 设置一个按钮
    pangolin::Var<bool> c_button("ui.laserData_Cluste", false, false);  // 设置一个按钮
    pangolin::Var<bool> d_button("ui.resetCameraView", false, false);
    pangolin::Var<int> a_int_slider("ui.thresholdGain", 10, 1, 50);     // 设置一个slider
    pangolin::Var<int> b_int_slider("ui.slideWindow", 5, 1, 15);        // 设置一个slider
    pangolin::Var<bool> a_checkBox("ui.showObstacle", false, true);     // 设置一个按钮
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    while( !pangolin::ShouldQuit() )
    {
        if (pangolin::Pushed(a_button)) {
            _showState = 0;
        } else if (pangolin::Pushed(b_button)) {
            _showState = 1;
        } else if (pangolin::Pushed(c_button)) {
            _showState = 2;
        } else if (pangolin::Pushed(d_button)) {
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1080,720,420,420,320,320,0.2,100));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-10,0,12, -10,0,0, pangolin::AxisY));
        }
        thresholdGain = a_int_slider.Get();
        slideWindowSize = b_int_slider.Get();
        switch (_showState) {
            case 0:
                _laserData_Show = laserData_Origin;
                break;
            case 1:
                // _laserData_Show = laserData_Filter;
                // break;
            case 2:
                _laserData_Show = laserData_Filter;
                break;
            default:
                break;
        }
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);


        glPointSize(15.0);
        glBegin( GL_POINTS );       
        glColor3f(1.0,0.647,0.0);
        glVertex3f(0, 0, 0);
        if (a_checkBox) {
            glVertex3f(obs.x, obs.y, 0);
        }
        glEnd();
        glBegin(GL_LINES);
        glLineWidth(2.0);
        glColor3f(0.0, 0.0, 0.0);
        for (int i = -15; i <= 15; ++i) {
            glVertex3d(i, -15, 0);
            glVertex3d(i, 15, 0);
            glVertex3d(-15, i, 0);
            glVertex3d(15, i, 0);
        }
        glEnd();

        if (_showState != 2) {
            glPointSize(5.0);
            glBegin( GL_POINTS ); 
            glColor3f(1.0,0.0,0.0);
            for (int i = 0; i < static_cast<int>(_laserData_Show.size()); ++i) {
                drawPoint(_laserData_Show[i]);
            }
            glEnd();  
        } else {
            int _pointIndex = 0;
            for (int i = 0; i <= laserData_Cluste.back(); ++i) {
                glPointSize(5.0);
                glBegin( GL_POINTS ); 
                glColor3f(colorLib[i % 12][0],colorLib[i % 12][1],colorLib[i % 12][2]);
                while (_pointIndex < static_cast<int>(_laserData_Show.size()) && 
                    laserData_Cluste[_pointIndex] == i) {
                    drawPoint(_laserData_Show[_pointIndex]);
                    ++_pointIndex;
                }
                glEnd(); 
            }
        }                  

        pangolin::FinishFrame();
        // usleep(10e6 / SCAN_SHOW_FPS);
        usleep(20000);
    }
    pangolin::DestroyWindow("LaserScan");
    return ;
}
#endif