#include "../include/view_handler.h"

namespace pangolin {

myHandler::myHandler(OpenGlRenderState& cam_state, ros::NodeHandle* n, AxisDirection enforce_up, float trans_scale, float zoom_fraction)
    : cam_state(&cam_state), _n(n), enforce_up(enforce_up), tf(trans_scale), zf(zoom_fraction), cameraspec(CameraSpecOpenGl), last_z(0.8)
{
    SetZero<3,1>(rot_center);
    last_press_t = std::chrono::steady_clock::now();
    pub_target_pos = _n->advertise<std_msgs::Float32MultiArray>("/target_pos", 1000);
    pub_key_order = _n->advertise<std_msgs::String>("/key_order", 1000);
}

bool myHandler::ValidWinDepth(GLprecision depth)
{
    return depth != 1;
}

void myHandler::PixelUnproject( View& view, GLprecision winx, GLprecision winy, GLprecision winz, GLprecision Pc[3])
{
    const GLint viewport[4] = {view.v.l,view.v.b,view.v.w,view.v.h};
    const pangolin::OpenGlMatrix proj = cam_state->GetProjectionMatrix();
    glUnProject(winx, winy, winz, Identity4d, proj.m, viewport, &Pc[0], &Pc[1], &Pc[2]);
}

void myHandler::GetPosNormal(pangolin::View& view, int winx, int winy, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision nw[3], GLprecision default_z)
{
    const int zl = (hwin*2+1);
    const int zsize = zl*zl;
    GLfloat zs[zsize];

    glReadBuffer(GL_FRONT);
    glReadPixels(winx-hwin,winy-hwin,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
    GLfloat mindepth = *(std::min_element(zs,zs+zsize));
    if(mindepth == 1) mindepth = (GLfloat)default_z;

    p[0] = winx; p[1] = winy; p[2] = mindepth;
    PixelUnproject(view, winx, winy, mindepth, Pc);

    const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();

    GLprecision T_wc[3*4];
    LieSE3from4x4(T_wc, mv.Inverse().m );
    LieApplySE3vec(Pw, T_wc, Pc);

    // Neighboring points in camera coordinates
    GLprecision Pl[3]; GLprecision Pr[3]; GLprecision Pb[3]; GLprecision Pt[3];
    PixelUnproject(view, winx-hwin, winy, zs[hwin*zl + 0],    Pl );
    PixelUnproject(view, winx+hwin, winy, zs[hwin*zl + zl-1], Pr );
    PixelUnproject(view, winx, winy-hwin, zs[hwin+1],         Pb );
    PixelUnproject(view, winx, winy+hwin, zs[zsize-(hwin+1)], Pt );
    
    // n = ((Pr-Pl).cross(Pt-Pb)).normalized();
    GLprecision PrmPl[3]; GLprecision PtmPb[3];
    MatSub<3,1>(PrmPl,Pr,Pl);
    MatSub<3,1>(PtmPb,Pt,Pb);

    GLprecision nc[3];
    CrossProduct(nc, PrmPl, PtmPb);
    Normalise<3>(nc);

    // T_wc is col major, so the rotation component is first.
    LieApplySO3(nw,T_wc,nc);
}

void myHandler::Mouse(View& display, MouseButton button, int x, int y, bool pressed, int button_state)
{
    // mouse down
    last_pos[0] = (float)x;
    last_pos[1] = (float)y;
    
    GLprecision T_nc[3*4];
    LieSetIdentity(T_nc);

    if( pressed ) {
        bool double_click = false;   
        auto cur_press_t=std::chrono::steady_clock::now();
        double dr_ms=std::chrono::duration<double,std::milli>(cur_press_t-last_press_t).count();
        
        if (last_press_button == button && dr_ms > 0 && dr_ms < 300) {
            double_click = true;
        }

        GetPosNormal(display,x,y,p,Pw,Pc,n,last_z);
    
        if( ValidWinDepth(p[2]) ) {
            last_z = p[2];
            std::copy(Pc,Pc+3,rot_center);
        }
        
        if( button == MouseWheelUp || button == MouseWheelDown) {
            LieSetTranslation<>(T_nc,rot_center);
            const GLprecision s = (button == MouseWheelUp ? -1.0 : 1.0) * zf;
            MatMul<3,1>(T_nc+(3*3), s);
            OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
            LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
        } else if (double_click && button == MouseButtonRight) {
            std_msgs::Float32MultiArray tar_pos;          
            tar_pos.data.emplace_back(Pw[0]);    
            tar_pos.data.emplace_back(Pw[1]);
            tar_pos.data.emplace_back(0);
            pub_target_pos.publish(tar_pos);
            ROS_INFO("Publish the swarm target postion:\n\tpos_x = %f, pox_y = %f", Pw[0], Pw[1]);
        }
        
        last_press_button = button;
        last_press_t = cur_press_t;
    }
    
}

void myHandler::MouseMotion(View& display, int x, int y, int button_state)
{
    const GLprecision rf = 0.01;
    const float delta[2] = { (float)x - last_pos[0], (float)y - last_pos[1] };
    const float mag = delta[0]*delta[0] + delta[1]*delta[1];
    
    if( mag < 50.0f*50.0f ) {
        OpenGlMatrix& mv = cam_state->GetModelViewMatrix();
        const GLprecision* up = AxisDirectionVector[enforce_up];
        GLprecision T_nc[3*4];
        LieSetIdentity(T_nc);
        bool rotation_changed = false;
        
        if( button_state == MouseButtonLeft ) {
            // Left Drag: in plane translate
            if( ValidWinDepth(last_z) ) {
                GLprecision np[3];
                PixelUnproject(display, x, y, last_z, np);
                const GLprecision t[] = { np[0] - rot_center[0], np[1] - rot_center[1], 0};
                LieSetTranslation<>(T_nc,t);
                std::copy(np,np+3,rot_center);
            } else {
                const GLprecision t[] = { -10*delta[0]*tf, 10*delta[1]*tf, 0};
                LieSetTranslation<>(T_nc,t);
            }
        } else if( button_state == (MouseButtonLeft | MouseButtonRight) ) {
            GLprecision T_2c[3*4];
            Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, delta[0]*rf);
            GLprecision mrotc[3];
            MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
            LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
            GLprecision T_n2[3*4];
            LieSetIdentity<>(T_n2);
            LieSetTranslation<>(T_n2,rot_center);
            LieMulSE3(T_nc, T_n2, T_2c );
            rotation_changed = true;
        }
        
        LieMul4x4bySE3<>(mv.m,T_nc,mv.m);
        
        if(enforce_up != AxisNone && rotation_changed) {
            EnforceUpT_cw(mv.m, up);
        }
    }
    
    last_pos[0] = (float)x;
    last_pos[1] = (float)y;
}

void myHandler::Keyboard(View&, unsigned char key, int x, int y, bool pressed)
{
    if (pressed) {
        std::string order_str;
        if (key == 'q' || key == '1') {
            order_str = "line";
            ROS_INFO("Switch formation : LINE");
        } else if (key == 'w' || key == '2') {
            order_str = "column";
            ROS_INFO("Switch formation : COLUMN");
        } else if (key == 'e' || key == '3') {
            order_str = "trig";
            ROS_INFO("Switch formation : TRIANGLE");
        } else if (key == 'a' || key == '4') {
            order_str = "avoid-yes";
            ROS_INFO("Open avoid mode");
        } else if (key == 's' || key == '5') {
            order_str = "avoid-no";
            ROS_INFO("Close avoid mode");
        }
        if (!order_str.empty()) {
            std_msgs::String kk_order;
            kk_order.data = order_str;
            pub_key_order.publish(kk_order);
        }
    }
}

}
