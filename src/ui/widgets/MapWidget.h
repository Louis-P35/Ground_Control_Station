#pragma once
#include <QWidget>
#include <QMap>
#include <QVector>
#include <QPixmap>
#include "backend/TelemetryState.h"

class QNetworkAccessManager;
class QNetworkReply;
class QPushButton;
class QCheckBox;

// ---------------------------------------------------------------------------
// MapWidget — satellite/street map view with real-time drone position overlay.
//
// Tiles are fetched from OpenStreetMap (XYZ tile scheme, Web Mercator projection)
// and cached on disk via QNetworkDiskCache. The drone is rendered as a circle
// with a directional arrow that follows the GPS heading. A red trail shows
// the drone's path since the session started.
//
// Interactions:
//   - Mouse wheel    : zoom in/out (anchor = cursor position)
//   - Left drag      : pan (disables follow mode)
//   - [⊕ Follow] btn : re-enable auto-centering on the drone
//   - [+] / [−] btn  : zoom in / zoom out
//   - Trail checkbox : toggle path display
// ---------------------------------------------------------------------------

class MapWidget : public QWidget {
    Q_OBJECT
public:
    explicit MapWidget(QWidget* parent = nullptr);

    // Called from MainWindow::onGpsReceived (UI thread).
    void updatePosition(const GpsData& d);

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void resizeEvent(QResizeEvent*) override;

private:
    // Web Mercator: lat/lon → world-pixel at given zoom level.
    // World pixel origin is top-left of the map; one tile = TILE_SIZE px.
    static QPointF latLonToWorld(double lat, double lon, int zoom);

    void    fetchTile(int z, int x, int y);
    QString tileKey(int z, int x, int y) const;
    void    applyZoom(int newZoom, QPointF anchorWidget);
    void    centerOnDrone();
    void    repositionOverlay();
    // Cancel all in-flight tile requests (called on zoom change to free slots immediately)
    void    cancelPendingRequests();

    // Drone state
    double m_lat     = 48.8566; // default: Paris until first GPS packet
    double m_lon     = 2.3522;
    float  m_heading = 0.0f;
    bool   m_hasFix  = false;
    bool   m_follow  = true;    // auto-center on drone

    // View state
    int    m_zoom       = 15;
    double m_centerPixX = 0.0;  // world-pixel coordinates of widget center
    double m_centerPixY = 0.0;

    // Pan drag state
    bool   m_dragging    = false;
    QPoint m_dragOrigin;
    double m_dragCenterX = 0.0;
    double m_dragCenterY = 0.0;

    // Network & tile cache
    QNetworkAccessManager*          m_nam = nullptr;
    QMap<QString, QPixmap>          m_tileCache;
    QMap<QString, QNetworkReply*>   m_pendingReplies; // key → in-flight reply

    // GPS trail — stores (lat, lon) pairs, capped at MAX_TRAIL entries
    QVector<QPointF> m_trail;

    // Tile layer
    bool m_satellite = false; // false = OSM street map, true = Esri satellite imagery

    // Overlay controls (absolute-positioned children, always on top)
    QPushButton* m_followBtn   = nullptr;
    QPushButton* m_zoomInBtn   = nullptr;
    QPushButton* m_zoomOutBtn  = nullptr;
    QPushButton* m_layerBtn    = nullptr;
    QCheckBox*   m_trailCheck  = nullptr;

    static constexpr int TILE_SIZE   = 256;
    static constexpr int MIN_ZOOM    = 2;
    static constexpr int MAX_ZOOM    = 19;
    static constexpr int MAX_CACHED  = 512;  // max in-memory tiles
    static constexpr int MAX_INFLIGHT = 6;   // max concurrent HTTP requests
    static constexpr int MAX_TRAIL   = 8000; // max trail points (~13 min at 10 Hz)
};
