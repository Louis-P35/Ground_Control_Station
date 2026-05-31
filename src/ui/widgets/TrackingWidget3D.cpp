#include "TrackingWidget3D.h"
#include <QMatrix4x4>
#include <QVector4D>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QtMath>
#include <cmath>
#include <algorithm>

// Fixed-function OpenGL is used here for consistency with DroneWidget3D: the
// drone model and lighting are identical, so the two views render the same craft.

namespace
{
    // Base follow pose, expressed as the world offset (0, +4 up, +8 behind).
    // This is the requested NED offset (0, −8, +4) reinterpreted so the camera
    // sits behind (south, +Z) and above (+Y) the drone, looking down at it.
    const float kBaseDistance  = std::sqrt(4.0f * 4.0f + 8.0f * 8.0f); // ≈ 8.944 m
    const float kBaseElevation = std::atan2(4.0f, 8.0f);               // ≈ 26.57°
    const float kBaseAzimuth   = 0.0f;

    // Easing time constant — gives roughly a 0.5 s settle (3–4 time constants).
    const float kEaseTau = 0.15f;

    // Drone is considered "near the edge" once its screen projection passes 85%
    // of the half-extent; the camera then backs off to re-frame it around 78%.
    const float kEdgeTrigger = 0.85f;
    const float kEdgeTarget  = 0.78f;
    const float kEdgeRelax   = 0.55f;
    const float kMaxBackoff  = 5.0f;

    float easeTo(float cur, float target, float alpha)
    {
        return cur + (target - cur) * alpha;
    }

    QVector3D easeTo(const QVector3D& cur, const QVector3D& target, float alpha)
    {
        return cur + (target - cur) * alpha;
    }
}

TrackingWidget3D::TrackingWidget3D(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(300, 300);

    // Seed the camera at the base follow pose.
    m_distance      = kBaseDistance;
    m_elevation     = kBaseElevation;
    m_azimuth       = kBaseAzimuth;
    m_userDistance  = kBaseDistance;
    m_userElevation = kBaseElevation;
    m_userAzimuth   = kBaseAzimuth;
}

// ---------------------------------------------------------------------------
// NED → OpenGL world conversion (see header for the full rationale).
//     world_x =  east        world_y = -down        world_z = -north
// ---------------------------------------------------------------------------
QVector3D TrackingWidget3D::nedToWorld(float north_m, float east_m, float down_m)
{
    return QVector3D(east_m, -down_m, -north_m);
}

// ---------------------------------------------------------------------------
// Telemetry inputs (called on the UI thread from MainWindow's queued slots)
// ---------------------------------------------------------------------------

void TrackingWidget3D::updateAttitude(const AttitudeData& d)
{
    // Identical axis remap to DroneWidget3D so both views agree on orientation:
    //   pitch up   → −OGL X      yaw right → +OGL Y      roll right → −OGL Z
    m_qw =  d.qw;
    m_qx = -d.qy;
    m_qy =  d.qz;
    m_qz = -d.qx;
    update();
}

void TrackingWidget3D::updatePosition(const PositionData& d)
{
    if (!d.valid)
    {
        return;
    }

    m_hasPosition = true;
    m_dronePos    = nedToWorld(d.north_m, d.east_m, d.down_m);

    // Append to the trail, dropping the oldest sample beyond the configured cap.
    m_trail.push_back(m_dronePos);
    while (static_cast<int>(m_trail.size()) > m_maxTrail)
    {
        m_trail.pop_front();
    }

    update();
}

// ---------------------------------------------------------------------------
// OpenGL setup
// ---------------------------------------------------------------------------

void TrackingWidget3D::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f); // Dark blue-grey background
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);

    // Soft directional + ambient lighting so the model stays readable from any
    // angle. The position (w = 0) makes GL_LIGHT0 a directional light; it is
    // re-specified each frame in world space after the view matrix is loaded.
    GLfloat lightAmb[]  = { 0.35f, 0.35f, 0.38f, 1.0f };
    GLfloat lightDiff[] = { 0.85f, 0.85f, 0.82f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiff);
    glEnable(GL_LIGHT0);

    // Blending for the fading trail.
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void TrackingWidget3D::resizeGL(int w, int h)
{
    m_viewW = std::max(1, w);
    m_viewH = std::max(1, h);
    glViewport(0, 0, m_viewW, m_viewH);
}

void TrackingWidget3D::paintGL()
{
    // ── Frame timing for time-based easing ──────────────────────────────────
    float dt = 0.016f;
    if (!m_initialised)
    {
        // First frame: snap the look-at onto the drone so there is no sweep-in.
        m_lookAt      = m_dronePos;
        m_initialised = true;
        m_frameTimer.start();
    }
    else
    {
        dt = static_cast<float>(m_frameTimer.restart()) / 1000.0f;
        dt = std::clamp(dt, 0.0f, 0.1f);
    }
    const float alpha = 1.0f - std::exp(-dt / kEaseTau);

    const bool manualActive = m_manualTimer.isValid() && m_manualTimer.elapsed() < 3000;

    // ── Smoothly trail the look-at point behind the true drone position ──────
    m_lookAt = easeTo(m_lookAt, m_dronePos, alpha);

    // ── Ease the orbit toward its target (manual pose or automatic pose) ────
    float targetAz   = manualActive ? m_userAzimuth   : kBaseAzimuth;
    float targetEl   = manualActive ? m_userElevation : kBaseElevation;
    float targetDist = manualActive ? m_userDistance  : kBaseDistance * m_autoScale;

    m_azimuth   = easeTo(m_azimuth,   targetAz,   alpha);
    m_elevation = easeTo(m_elevation, targetEl,   alpha);
    m_distance  = easeTo(m_distance,  targetDist, alpha);

    // ── Build the camera offset from spherical coordinates ──────────────────
    const float horiz = m_distance * std::cos(m_elevation);
    const QVector3D offset(horiz * std::sin(m_azimuth),
                           m_distance * std::sin(m_elevation),
                           horiz * std::cos(m_azimuth));
    const QVector3D eye = m_lookAt + offset;

    // ── Projection and view matrices ────────────────────────────────────────
    const float aspect = static_cast<float>(m_viewW) / static_cast<float>(m_viewH);
    QMatrix4x4 proj;
    proj.perspective(45.0f, aspect, 0.1f, 300.0f);

    QMatrix4x4 view;
    view.lookAt(eye, m_lookAt, QVector3D(0.0f, 1.0f, 0.0f));

    // ── Adaptive back-off: keep the (real) drone inside the frame ───────────
    // Project the true drone position to normalised device coordinates. When it
    // drifts past the edge threshold, grow the back-off so the FOV re-covers it.
    if (!manualActive)
    {
        const QVector4D clip = proj * view * QVector4D(m_dronePos, 1.0f);
        if (clip.w() > 1e-4f)
        {
            const float maxAbs = std::max(std::abs(clip.x() / clip.w()),
                                          std::abs(clip.y() / clip.w()));
            float targetScale = m_autoScale;
            if (maxAbs > kEdgeTrigger)
            {
                targetScale = m_autoScale * (maxAbs / kEdgeTarget);
            }
            else if (maxAbs < kEdgeRelax)
            {
                targetScale = 1.0f;
            }
            targetScale  = std::clamp(targetScale, 1.0f, kMaxBackoff);
            m_autoScale  = easeTo(m_autoScale, targetScale, alpha);
        }
    }

    // ── Render ──────────────────────────────────────────────────────────────
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.constData());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.constData());

    // World-fixed directional light (set while modelview == view).
    GLfloat lightDir[] = { 0.4f, 1.0f, 0.6f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightDir);

    drawGrid();
    drawTrail();
    drawDrone();

    // ── Keep animating until everything has settled ─────────────────────────
    // This drives the easing without a fixed-rate timer: repaints stop once the
    // camera is at rest, and resume on the next telemetry update or user input.
    const bool moving =
        std::abs(m_azimuth   - targetAz)   > 1e-4f ||
        std::abs(m_elevation - targetEl)   > 1e-4f ||
        std::abs(m_distance  - targetDist) > 1e-3f ||
        (m_lookAt - m_dronePos).lengthSquared() > 1e-6f ||
        manualActive ||
        (m_manualTimer.isValid() && m_manualTimer.elapsed() < 3200);

    if (moving)
    {
        update();
    }
}

// ---------------------------------------------------------------------------
// Scene elements
// ---------------------------------------------------------------------------

void TrackingWidget3D::drawGrid()
{
    // 20×20 m grid, 1 m spacing, centred on the origin, on the ground plane y=0.
    glDisable(GL_LIGHTING);
    glColor3f(0.32f, 0.32f, 0.36f);
    glBegin(GL_LINES);
    const int half = 10;
    for (int i = -half; i <= half; ++i)
    {
        const float c = static_cast<float>(i);
        // Lines running along Z (north/south)
        glVertex3f(c, 0.0f, -static_cast<float>(half));
        glVertex3f(c, 0.0f,  static_cast<float>(half));
        // Lines running along X (east/west)
        glVertex3f(-static_cast<float>(half), 0.0f, c);
        glVertex3f( static_cast<float>(half), 0.0f, c);
    }
    glEnd();
}

void TrackingWidget3D::drawTrail()
{
    // No trail until we have a real position fix.
    if (!m_hasPosition || m_trail.size() < 2)
    {
        return;
    }

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE); // Trail should not occlude itself or the drone

    glBegin(GL_LINE_STRIP);
    const int   n    = static_cast<int>(m_trail.size());
    const float maxA = 0.9f; // Newest segment opacity
    for (int i = 0; i < n; ++i)
    {
        // Opacity ramps from transparent (oldest) to opaque (newest).
        const float t = static_cast<float>(i) / static_cast<float>(n - 1);
        glColor4f(0.2f, 0.5f, 1.0f, t * maxA);
        const QVector3D& p = m_trail[static_cast<size_t>(i)];
        glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void TrackingWidget3D::drawDrone()
{
    glEnable(GL_LIGHTING);
    glPushMatrix();

    // Translate to the drone position (real fix, or the floating default centre).
    glTranslatef(m_dronePos.x(), m_dronePos.y(), m_dronePos.z());

    // Apply the attitude quaternion.
    float rot[16];
    quaternionToMatrix(m_qw, m_qx, m_qy, m_qz, rot);
    glMultMatrixf(rot);

    drawBody();

    // 4 arms at 45°, 135°, 225°, 315° — identical model to DroneWidget3D.
    const float angles[4] = { 45.0f, 135.0f, 225.0f, 315.0f };
    const float colors[4][3] =
    {
        {0.2f, 0.8f, 0.2f},
        {0.2f, 0.4f, 0.9f},
        {0.9f, 0.2f, 0.2f},
        {0.9f, 0.8f, 0.1f},
    };
    const float armLen = 0.8f;
    for (int i = 0; i < 4; ++i)
    {
        const float rad = angles[i] * static_cast<float>(M_PI) / 180.0f;
        glColor3fv(colors[i]);
        drawArm(rad, armLen, 0.05f);
        const float px = armLen * std::cos(rad);
        const float pz = armLen * std::sin(rad);
        drawRotor(px, 0.05f, pz, 0.25f);
    }

    glPopMatrix();
}

void TrackingWidget3D::drawBody()
{
    glColor3f(0.5f, 0.5f, 0.5f);
    const float s = 0.2f;
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);  glVertex3f(-s, s, -s); glVertex3f(s, s, -s); glVertex3f(s, s, s); glVertex3f(-s, s, s);
    glNormal3f(0, -1, 0); glVertex3f(-s, -s, s); glVertex3f(s, -s, s); glVertex3f(s, -s, -s); glVertex3f(-s, -s, -s);
    glNormal3f(0, 0, 1);  glVertex3f(-s, -s, s); glVertex3f(s, -s, s); glVertex3f(s, s, s); glVertex3f(-s, s, s);
    glNormal3f(0, 0, -1); glVertex3f(s, -s, -s); glVertex3f(-s, -s, -s); glVertex3f(-s, s, -s); glVertex3f(s, s, -s);
    glNormal3f(-1, 0, 0); glVertex3f(-s, -s, -s); glVertex3f(-s, -s, s); glVertex3f(-s, s, s); glVertex3f(-s, s, -s);
    glNormal3f(1, 0, 0);  glVertex3f(s, -s, s); glVertex3f(s, -s, -s); glVertex3f(s, s, -s); glVertex3f(s, s, s);
    glEnd();
}

void TrackingWidget3D::drawArm(float angleRad, float armLen, float thickness)
{
    const float cx = armLen * 0.5f * std::cos(angleRad);
    const float cz = armLen * 0.5f * std::sin(angleRad);

    glPushMatrix();
    glTranslatef(cx, 0, cz);
    glRotatef(-angleRad * 180.0f / static_cast<float>(M_PI), 0, 1, 0);
    const float hw = thickness, hh = thickness * 0.5f, hl = armLen * 0.5f;
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-hl, hh, -hw); glVertex3f(hl, hh, -hw); glVertex3f(hl, hh, hw); glVertex3f(-hl, hh, hw);
    glNormal3f(0, -1, 0);
    glVertex3f(-hl, -hh, hw); glVertex3f(hl, -hh, hw); glVertex3f(hl, -hh, -hw); glVertex3f(-hl, -hh, -hw);
    glNormal3f(0, 0, 1);
    glVertex3f(-hl, -hh, hw); glVertex3f(hl, -hh, hw); glVertex3f(hl, hh, hw); glVertex3f(-hl, hh, hw);
    glNormal3f(0, 0, -1);
    glVertex3f(hl, -hh, -hw); glVertex3f(-hl, -hh, -hw); glVertex3f(-hl, hh, -hw); glVertex3f(hl, hh, -hw);
    glEnd();
    glPopMatrix();
}

void TrackingWidget3D::drawRotor(float x, float y, float z, float radius)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    const int segs = 24;
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= segs; ++i)
    {
        const float a = 2.0f * static_cast<float>(M_PI) * i / segs;
        glVertex3f(radius * std::cos(a), 0, radius * std::sin(a));
    }
    glEnd();
    glPopMatrix();
}

void TrackingWidget3D::quaternionToMatrix(float qw, float qx, float qy, float qz, float m[16]) const
{
    const float x2 = qx * qx, y2 = qy * qy, z2 = qz * qz;
    const float xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const float wx = qw * qx, wy = qw * qy, wz = qw * qz;

    m[0]  = 1 - 2 * (y2 + z2); m[1]  = 2 * (xy + wz);     m[2]  = 2 * (xz - wy);     m[3]  = 0;
    m[4]  = 2 * (xy - wz);     m[5]  = 1 - 2 * (x2 + z2); m[6]  = 2 * (yz + wx);     m[7]  = 0;
    m[8]  = 2 * (xz + wy);     m[9]  = 2 * (yz - wx);     m[10] = 1 - 2 * (x2 + y2); m[11] = 0;
    m[12] = 0;                 m[13] = 0;                 m[14] = 0;                 m[15] = 1;
}

// ---------------------------------------------------------------------------
// Manual camera control — wheel zoom and right-drag orbit.
// Each interaction restarts the 3-second override window; the automatic follow
// eases back in once the window expires.
// ---------------------------------------------------------------------------

void TrackingWidget3D::wheelEvent(QWheelEvent* e)
{
    // Seed the manual targets from the current pose if no override is active.
    const bool active = m_manualTimer.isValid() && m_manualTimer.elapsed() < 3000;
    if (!active)
    {
        m_userAzimuth   = m_azimuth;
        m_userElevation = m_elevation;
        m_userDistance  = m_distance;
    }

    // 120 units per notch; positive delta zooms in (shorter distance).
    const float steps  = e->angleDelta().y() / 120.0f;
    m_userDistance     = std::clamp(m_userDistance * std::pow(0.9f, steps), 2.0f, 60.0f);

    m_manualTimer.restart();
    update();
}

void TrackingWidget3D::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton)
    {
        const bool active = m_manualTimer.isValid() && m_manualTimer.elapsed() < 3000;
        if (!active)
        {
            m_userAzimuth   = m_azimuth;
            m_userElevation = m_elevation;
            m_userDistance  = m_distance;
        }
        m_dragging  = true;
        m_lastMouse = e->pos();
        m_manualTimer.restart();
    }
}

void TrackingWidget3D::mouseMoveEvent(QMouseEvent* e)
{
    if (!m_dragging)
    {
        return;
    }

    const QPoint delta = e->pos() - m_lastMouse;
    m_lastMouse = e->pos();

    m_userAzimuth  += delta.x() * 0.01f;
    m_userElevation = std::clamp(m_userElevation - delta.y() * 0.01f,
                                 qDegreesToRadians(5.0f),
                                 qDegreesToRadians(85.0f));

    m_manualTimer.restart();
    update();
}

void TrackingWidget3D::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton)
    {
        m_dragging = false;
    }
}
