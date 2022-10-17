#pragma once

#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <pangolin/pangolin.h>

namespace pangolin {

struct PANGOLIN_EXPORT myHandler : Handler
{
    myHandler(OpenGlRenderState& cam_state, ros::NodeHandle* n, AxisDirection enforce_up=AxisNone, float trans_scale=0.01f, float zoom_fraction= PANGO_DFLT_HANDLER3D_ZF);

    virtual bool ValidWinDepth(GLprecision depth);
    virtual void PixelUnproject( View& view, GLprecision winx, GLprecision winy, GLprecision winz, GLprecision Pc[3]);
    virtual void GetPosNormal(View& view, int x, int y, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision nw[3], GLprecision default_z = 1.0);

    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
    void MouseMotion(View&, int x, int y, int button_state);    

private:
    OpenGlRenderState* cam_state;
    const static int hwin = 8;
    AxisDirection enforce_up;
    float tf; // translation factor
    float zf; // zoom fraction
    CameraSpec cameraspec;
    GLprecision last_z;
    float last_pos[2];
    GLprecision rot_center[3];
    
    GLprecision p[3];
    GLprecision Pw[3];
    GLprecision Pc[3];
    GLprecision n[3];

    MouseButton last_press_button;
    std::chrono::time_point<std::chrono::steady_clock> last_press_t;

    ros::NodeHandle* _n;
    ros::Publisher pub_target_pos;
};

}