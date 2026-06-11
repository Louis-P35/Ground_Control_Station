#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QElapsedTimer>
#include <QVector3D>
#include <QMatrix4x4>
#include <QPointF>
#include <deque>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// TrackingWidget3D — third-person "follow" view of the drone in a local scene.
//
// Renders, using the same fixed-function OpenGL pipeline as DroneWidget3D:
//   • a flat 20×20 m ground grid (1 m spacing) centred on the origin,
//   • the quadcopter model, oriented by the live attitude quaternion and
//     translated to its NWU position (or floating at the scene centre when no
//     position fix is available yet),
//   • a fading blue movement trail of the last N positions.
//
// The camera follows the drone from behind and above. It smoothly trails the
// motion, automatically backs off when the drone drifts toward the view edge,
// and supports manual wheel-zoom and right-drag orbit that temporarily override
// the automatic follow for 3 seconds before easing back.
//
// Frame conversion — NWU (north, west, up) → OpenGL world (X right, Y up,
// Z toward camera):
//     world_x = -west_m         (West  → −X, i.e. East is OpenGL +X)
//     world_y =  up_m           (Up    → +Y)
//     world_z = -north_m        (North → −Z, into the screen)
// The attitude quaternion is remapped with the exact same axis convention as
// DroneWidget3D so both views agree on orientation.
//
// Threading: updateAttitude() / updatePosition() are invoked on the UI thread
// from MainWindow's queued telemetry slots (the cross-thread hop is the queued
// signal/slot connection). The widget never polls; repaints are driven by
// incoming telemetry and, while the camera is still easing, by self-scheduled
// updates that stop once the motion settles.
// ---------------------------------------------------------------------------

class TrackingWidget3D : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit TrackingWidget3D(QWidget* parent = nullptr);

    void updateAttitude(const AttitudeData& d);
    void updatePosition(const PositionData& d);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void wheelEvent(QWheelEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

private:
    // ── Scene drawing ──────────────────────────────────────────────────────
    // Not const: the fixed-function GL calls go through QOpenGLFunctions, whose
    // methods are non-const because they mutate the bound GL state.
    void drawGrid();
    void drawTrail();
    void drawDrone();
    void drawDroneAxes();  // NWU reference triad + N/W/U letters on the drone (GL)
    void drawArm(float angleRad, float armLen, float thickness);
    void drawRotor(float x, float y, float z, float radius);
    void drawBody();
    void quaternionToMatrix(float qw, float qx, float qy, float qz, float m[16]) const;

    // ── NWU → OpenGL world conversion ──────────────────────────────────────
    static QVector3D nwuToWorld(float north_m, float west_m, float up_m);

    // ── Attitude (already remapped to OpenGL axes) ─────────────────────────
    float m_qw = 1, m_qx = 0, m_qy = 0, m_qz = 0;

    // ── Position ───────────────────────────────────────────────────────────
    bool      m_hasPosition = false;             // True once a real fix arrived
    QVector3D m_dronePos    = {0.0f, 2.0f, 0.0f}; // World coords; default = floating centre
    std::deque<QVector3D> m_trail;               // World coords, oldest → newest
    int       m_maxTrail    = 300;               // Configurable trail length

    // ── Camera (spherical offset around the look-at point) ─────────────────
    // Base follow pose: behind (south, +Z) and above (+Y) the drone. These
    // angles reproduce the requested NED offset (0, −8, +4) reinterpreted so
    // the camera sits behind and above looking down at the drone.
    QVector3D m_lookAt      = {0.0f, 2.0f, 0.0f}; // Smoothed point the camera tracks
    float     m_azimuth     = 0.0f;               // Current orbit yaw   (rad)
    float     m_elevation   = 0.0f;               // Current orbit pitch (rad)
    float     m_distance    = 0.0f;               // Current camera distance (m)
    float     m_autoScale   = 1.0f;               // Back-off multiplier when near edge

    // Manual override state — user-driven azimuth/elevation/distance
    float         m_userAzimuth   = 0.0f;
    float         m_userElevation = 0.0f;
    float         m_userDistance  = 0.0f;
    QElapsedTimer m_manualTimer;                  // Time since last manual input

    // Right-drag orbit bookkeeping
    bool   m_dragging = false;
    QPoint m_lastMouse;

    // Frame timing for time-based easing (no fixed-rate polling)
    QElapsedTimer m_frameTimer;
    bool          m_initialised = false;
    int           m_viewW = 1, m_viewH = 1;

    // View matrix cached each frame; drawDroneAxes() uses its rows as the camera
    // right/up vectors to billboard the N/E/D letters toward the viewer.
    QMatrix4x4 m_view;
};
