#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// DroneWidget3D — renders a simple quadcopter model using OpenGL.
// The drone rotates according to the live attitude quaternion.
// Camera is fixed; only the model matrix changes.
// ---------------------------------------------------------------------------

class DroneWidget3D : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit DroneWidget3D(QWidget* parent = nullptr);

    void updateAttitude(const AttitudeData& d);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    // Quaternion (updated from telemetry)
    float m_qw = 1, m_qx = 0, m_qy = 0, m_qz = 0;

    // Draw helpers
    void drawArm(float angleRad, float armLen, float thickness);
    void drawRotor(float x, float y, float z, float radius);
    void drawBody();

    // Convert quaternion to 4x4 rotation matrix (column-major for OpenGL)
    void quaternionToMatrix(float qw, float qx, float qy, float qz, float mat[16]) const;
};
