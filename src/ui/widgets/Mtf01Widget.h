#pragma once
#include <QWidget>
#include <QLabel>
#include "backend/TelemetryState.h"

class Mtf01Widget : public QWidget {
    Q_OBJECT
public:
    explicit Mtf01Widget(QWidget* parent = nullptr);
    void updateData(const Mtf01Data& d);

private:
    QLabel* m_distance = nullptr;
    QLabel* m_flowX    = nullptr;
    QLabel* m_flowY    = nullptr;
    QLabel* m_quality  = nullptr;
};
