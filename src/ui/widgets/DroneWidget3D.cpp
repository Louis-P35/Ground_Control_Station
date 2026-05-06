#include "DroneWidget3D.h"
#include <QOpenGLFunctions>
#include <QtMath>
#include <cmath>
#include <cstring>

// Use deprecated fixed-function OpenGL for simplicity (no shader setup required)
// This is sufficient for a lightweight drone visualiser.

DroneWidget3D::DroneWidget3D(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(200, 200);
}

void DroneWidget3D::updateAttitude(const AttitudeData& d) {
    m_qw = d.qw; m_qx = d.qx; m_qy = d.qy; m_qz = d.qz;
    update(); // Schedule repaint
}

void DroneWidget3D::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Dark background
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);

    // Light setup
    GLfloat lightPos[]   = { 2.0f,  4.0f, 3.0f, 1.0f };
    GLfloat lightAmb[]   = { 0.2f,  0.2f, 0.2f, 1.0f };
    GLfloat lightDiff[]  = { 0.8f,  0.8f, 0.8f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiff);
}

void DroneWidget3D::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = h > 0 ? static_cast<float>(w) / h : 1.0f;
    // Simple perspective: ~60° FOV
    float f = 1.0f / std::tan(30.0f * M_PI / 180.0f);
    float near = 0.1f, far = 100.0f;
    float proj[16] = {};
    proj[0]  = f / aspect;
    proj[5]  = f;
    proj[10] = (far + near) / (near - far);
    proj[11] = -1.0f;
    proj[14] = (2.0f * far * near) / (near - far);
    glLoadMatrixf(proj);
}

void DroneWidget3D::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Fixed camera — closer than before so the drone fills the widget better
    float eyeX = 0, eyeY = 1.2f, eyeZ = 2.4f;
    float cenX = 0, cenY = 0, cenZ = 0;
    float upX  = 0, upY  = 1, upZ  = 0;
    float fx = cenX-eyeX, fy = cenY-eyeY, fz = cenZ-eyeZ;
    float flen = std::sqrt(fx*fx+fy*fy+fz*fz);
    fx/=flen; fy/=flen; fz/=flen;
    float rx = fy*upZ - fz*upY, ry = fz*upX - fx*upZ, rz = fx*upY - fy*upX;
    float rlen = std::sqrt(rx*rx+ry*ry+rz*rz);
    rx/=rlen; ry/=rlen; rz/=rlen;
    float ux = ry*fz-rz*fy, uy = rz*fx-rx*fz, uz = rx*fy-ry*fx;
    float view[16] = {
        rx, ux, -fx, 0,
        ry, uy, -fy, 0,
        rz, uz, -fz, 0,
        -(rx*eyeX+ry*eyeY+rz*eyeZ), -(ux*eyeX+uy*eyeY+uz*eyeZ), (fx*eyeX+fy*eyeY+fz*eyeZ), 1
    };
    glLoadMatrixf(view);

    // Apply drone rotation from quaternion
    float rot[16];
    quaternionToMatrix(m_qw, m_qx, m_qy, m_qz, rot);
    glMultMatrixf(rot);

    // Draw frame body
    drawBody();

    // Draw 4 arms at 45°, 135°, 225°, 315°
    float angles[4] = { 45.0f, 135.0f, 225.0f, 315.0f };
    // Motor colors: front-left=green, front-right=blue, back-left=red, back-right=yellow
    float colors[4][3] = {
        {0.2f, 0.8f, 0.2f},
        {0.2f, 0.4f, 0.9f},
        {0.9f, 0.2f, 0.2f},
        {0.9f, 0.8f, 0.1f},
    };
    float armLen = 0.8f;
    for (int i = 0; i < 4; ++i) {
        float rad = angles[i] * static_cast<float>(M_PI) / 180.0f;
        glColor3fv(colors[i]);
        drawArm(rad, armLen, 0.05f);
        float px = armLen * std::cos(rad);
        float pz = armLen * std::sin(rad);
        drawRotor(px, 0.05f, pz, 0.25f);
    }
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

void DroneWidget3D::drawBody() {
    // Central box
    glColor3f(0.5f, 0.5f, 0.5f);
    float s = 0.2f;
    glBegin(GL_QUADS);
    // Top
    glNormal3f(0,1,0); glVertex3f(-s,s,-s); glVertex3f(s,s,-s); glVertex3f(s,s,s); glVertex3f(-s,s,s);
    // Bottom
    glNormal3f(0,-1,0); glVertex3f(-s,-s,s); glVertex3f(s,-s,s); glVertex3f(s,-s,-s); glVertex3f(-s,-s,-s);
    // Front
    glNormal3f(0,0,1); glVertex3f(-s,-s,s); glVertex3f(s,-s,s); glVertex3f(s,s,s); glVertex3f(-s,s,s);
    // Back
    glNormal3f(0,0,-1); glVertex3f(s,-s,-s); glVertex3f(-s,-s,-s); glVertex3f(-s,s,-s); glVertex3f(s,s,-s);
    // Left
    glNormal3f(-1,0,0); glVertex3f(-s,-s,-s); glVertex3f(-s,-s,s); glVertex3f(-s,s,s); glVertex3f(-s,s,-s);
    // Right
    glNormal3f(1,0,0); glVertex3f(s,-s,s); glVertex3f(s,-s,-s); glVertex3f(s,s,-s); glVertex3f(s,s,s);
    glEnd();
}

void DroneWidget3D::drawArm(float angleRad, float armLen, float thickness) {
    // Draw a rectangular arm from origin toward (cos(a), 0, sin(a))
    float cx = armLen * 0.5f * std::cos(angleRad);
    float cz = armLen * 0.5f * std::sin(angleRad);

    glPushMatrix();
    glTranslatef(cx, 0, cz);
    glRotatef(-angleRad * 180.0f / static_cast<float>(M_PI), 0, 1, 0);
    float hw = thickness, hh = thickness * 0.5f, hl = armLen * 0.5f;
    glBegin(GL_QUADS);
    glNormal3f(0,1,0);
    glVertex3f(-hl, hh,-hw); glVertex3f(hl, hh,-hw); glVertex3f(hl, hh, hw); glVertex3f(-hl, hh, hw);
    glNormal3f(0,-1,0);
    glVertex3f(-hl,-hh, hw); glVertex3f(hl,-hh, hw); glVertex3f(hl,-hh,-hw); glVertex3f(-hl,-hh,-hw);
    glNormal3f(0,0,1);
    glVertex3f(-hl,-hh, hw); glVertex3f(hl,-hh, hw); glVertex3f(hl, hh, hw); glVertex3f(-hl, hh, hw);
    glNormal3f(0,0,-1);
    glVertex3f( hl,-hh,-hw); glVertex3f(-hl,-hh,-hw); glVertex3f(-hl, hh,-hw); glVertex3f( hl, hh,-hw);
    glEnd();
    glPopMatrix();
}

void DroneWidget3D::drawRotor(float x, float y, float z, float radius) {
    // Draw a flat disc
    glPushMatrix();
    glTranslatef(x, y, z);
    int segs = 24;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= segs; ++i) {
        float a = 2.0f * static_cast<float>(M_PI) * i / segs;
        glVertex3f(radius * std::cos(a), 0, radius * std::sin(a));
    }
    glEnd();
    glPopMatrix();
}

// ---------------------------------------------------------------------------
// Quaternion → column-major 4x4 rotation matrix
// ---------------------------------------------------------------------------
void DroneWidget3D::quaternionToMatrix(float qw, float qx, float qy, float qz, float m[16]) const {
    float x2 = qx*qx, y2 = qy*qy, z2 = qz*qz;
    float xy = qx*qy, xz = qx*qz, yz = qy*qz;
    float wx = qw*qx, wy = qw*qy, wz = qw*qz;

    m[0]  = 1-2*(y2+z2); m[1]  = 2*(xy+wz);  m[2]  = 2*(xz-wy);  m[3]  = 0;
    m[4]  = 2*(xy-wz);   m[5]  = 1-2*(x2+z2); m[6]  = 2*(yz+wx);  m[7]  = 0;
    m[8]  = 2*(xz+wy);   m[9]  = 2*(yz-wx);   m[10] = 1-2*(x2+y2); m[11] = 0;
    m[12] = 0;            m[13] = 0;            m[14] = 0;            m[15] = 1;
}
