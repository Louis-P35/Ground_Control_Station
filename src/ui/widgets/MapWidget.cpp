#include "MapWidget.h"
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QPushButton>
#include <QCheckBox>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QNetworkDiskCache>
#include <QStandardPaths>
#include <QUrl>
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Web Mercator projection (EPSG:3857 / OSM tile scheme)
//
// At zoom level z the world is divided into 2^z × 2^z tiles of TILE_SIZE px.
// World-pixel origin is at the top-left corner of the map (northwest).
// ---------------------------------------------------------------------------
QPointF MapWidget::latLonToWorld(double lat, double lon, int zoom) {
    double n   = std::pow(2.0, zoom);
    double x   = (lon + 180.0) / 360.0 * n * TILE_SIZE;
    double rad = lat * M_PI / 180.0;
    // Mercator Y formula; clamped near poles to avoid infinity
    double sinLat = std::sin(rad);
    sinLat = std::clamp(sinLat, -0.9999, 0.9999);
    double y = (1.0 - std::log((1.0 + sinLat) / (1.0 - sinLat)) / (2.0 * M_PI)) / 2.0 * n * TILE_SIZE;
    return {x, y};
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
MapWidget::MapWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(300, 200);

    // Disk-backed network cache — tiles are reused across sessions
    m_nam = new QNetworkAccessManager(this);
    auto* diskCache = new QNetworkDiskCache(this);
    diskCache->setCacheDirectory(
        QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/maptiles");
    diskCache->setMaximumCacheSize(200LL * 1024 * 1024); // 200 MB
    m_nam->setCache(diskCache);

    // Initialize view center on default position (overwritten on first GPS packet)
    QPointF wp = latLonToWorld(m_lat, m_lon, m_zoom);
    m_centerPixX = wp.x();
    m_centerPixY = wp.y();

    // --- Overlay buttons (children of this widget, absolute-positioned) ---
    auto mkBtn = [this](const QString& label) -> QPushButton* {
        auto* b = new QPushButton(label, this);
        b->setFixedHeight(28);
        b->setStyleSheet(
            "QPushButton { background: rgba(20,20,20,210); color: white; "
            "border: 1px solid #555; padding: 2px 6px; "
            "border-radius: 3px; font-size: 11px; }"
            "QPushButton:hover { background: rgba(60,60,60,230); }");
        return b;
    };
    m_followBtn  = mkBtn("⊕ Follow");
    m_zoomInBtn  = mkBtn("+");
    m_zoomOutBtn = mkBtn("−");
    m_followBtn ->setFixedWidth(76);
    m_zoomInBtn ->setFixedWidth(32);
    m_zoomOutBtn->setFixedWidth(32);

    connect(m_followBtn, &QPushButton::clicked, this, [this]() {
        m_follow = true;
        if (m_hasFix) centerOnDrone();
        update();
    });
    connect(m_zoomInBtn,  &QPushButton::clicked, this,
            [this]() { applyZoom(m_zoom + 1, {width() / 2.0, height() / 2.0}); });
    connect(m_zoomOutBtn, &QPushButton::clicked, this,
            [this]() { applyZoom(m_zoom - 1, {width() / 2.0, height() / 2.0}); });

    // Trail toggle checkbox
    m_trailCheck = new QCheckBox("Trail", this);
    m_trailCheck->setChecked(true);
    m_trailCheck->setFixedSize(70, 28);
    m_trailCheck->setStyleSheet(
        "QCheckBox { color: white; background: rgba(20,20,20,210); "
        "padding: 2px 6px; border: 1px solid #555; border-radius: 3px; font-size: 11px; }"
        "QCheckBox::indicator { width: 12px; height: 12px; border: 1px solid #888; "
        "border-radius: 2px; background: #333; }"
        "QCheckBox::indicator:checked { background: #c83232; border-color: #e05050; }");
    connect(m_trailCheck, &QCheckBox::toggled, this, [this](bool) { update(); });
}

// ---------------------------------------------------------------------------
// Public slot
// ---------------------------------------------------------------------------
void MapWidget::updatePosition(const GpsData& d) {
    m_lat     = d.latitude;
    m_lon     = d.longitude;
    m_heading = d.heading_deg;
    m_hasFix  = (d.fix_type >= 1);

    if (m_hasFix) {
        // Append to trail only when position actually changed (avoids pile-up of duplicates)
        QPointF pos(m_lat, m_lon);
        if (m_trail.isEmpty() || m_trail.last() != pos) {
            m_trail.append(pos);
            // Keep trail within memory budget by removing the oldest point
            if (m_trail.size() > MAX_TRAIL)
                m_trail.removeFirst();
        }
    }

    if (m_follow && m_hasFix)
        centerOnDrone();
    update();
}

void MapWidget::centerOnDrone() {
    QPointF wp = latLonToWorld(m_lat, m_lon, m_zoom);
    m_centerPixX = wp.x();
    m_centerPixY = wp.y();
}

// ---------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------
void MapWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    int W = width(), H = height();
    if (W < 2 || H < 2) return;

    // --- Determine visible tile range ---
    int txMin = static_cast<int>(std::floor((m_centerPixX - W / 2.0) / TILE_SIZE));
    int tyMin = static_cast<int>(std::floor((m_centerPixY - H / 2.0) / TILE_SIZE));
    int txMax = static_cast<int>(std::floor((m_centerPixX + W / 2.0) / TILE_SIZE));
    int tyMax = static_cast<int>(std::floor((m_centerPixY + H / 2.0) / TILE_SIZE));

    int tileCount = 1 << m_zoom; // 2^zoom tiles per axis

    for (int ty = tyMin; ty <= tyMax; ++ty) {
        for (int tx = txMin; tx <= txMax; ++tx) {
            // Widget-space top-left corner of this tile
            int sx = static_cast<int>(std::round(tx * TILE_SIZE - m_centerPixX + W / 2.0));
            int sy = static_cast<int>(std::round(ty * TILE_SIZE - m_centerPixY + H / 2.0));

            // Longitude wraps; latitude doesn't
            int cx = ((tx % tileCount) + tileCount) % tileCount;
            if (ty < 0 || ty >= tileCount) {
                p.fillRect(sx, sy, TILE_SIZE, TILE_SIZE, QColor(30, 30, 30));
                continue;
            }

            QString key = tileKey(m_zoom, cx, ty);
            if (m_tileCache.contains(key)) {
                p.drawPixmap(sx, sy, m_tileCache[key]);
            } else {
                // Placeholder while tile loads
                p.fillRect(sx, sy, TILE_SIZE, TILE_SIZE, QColor(28, 28, 28));
                p.setPen(QColor(50, 50, 50));
                p.drawRect(sx, sy, TILE_SIZE - 1, TILE_SIZE - 1);
                fetchTile(m_zoom, cx, ty);
            }
        }
    }

    // --- GPS trail ---
    if (m_trailCheck->isChecked() && m_trail.size() >= 2) {
        QPolygonF poly;
        poly.reserve(m_trail.size());
        for (const QPointF& pt : m_trail) {
            QPointF wp = latLonToWorld(pt.x(), pt.y(), m_zoom);
            poly << QPointF(wp.x() - m_centerPixX + W / 2.0,
                            wp.y() - m_centerPixY + H / 2.0);
        }
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QPen(QColor(210, 40, 40, 200), 2.5,
                      Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(poly);
    }

    // --- Drone marker ---
    if (m_hasFix) {
        QPointF droneWorld = latLonToWorld(m_lat, m_lon, m_zoom);
        float dx = static_cast<float>(droneWorld.x() - m_centerPixX + W / 2.0);
        float dy = static_cast<float>(droneWorld.y() - m_centerPixY + H / 2.0);

        p.setRenderHint(QPainter::Antialiasing);
        p.save();
        p.translate(dx, dy);
        p.rotate(static_cast<double>(m_heading)); // 0° = north = arrow pointing up

        // Drop shadow
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 0, 0, 70));
        p.drawEllipse(QPointF(2, 2), 9, 9);

        // Body circle
        p.setBrush(QColor(30, 120, 255, 220));
        p.setPen(QPen(Qt::white, 1.5));
        p.drawEllipse(QPointF(0, 0), 8, 8);

        // Directional arrow pointing toward current heading
        QPainterPath arrow;
        arrow.moveTo(0,  -18); // tip
        arrow.lineTo(5,  -8);  // right base
        arrow.lineTo(-5, -8);  // left base
        arrow.closeSubpath();
        p.setBrush(QColor(30, 120, 255, 230));
        p.setPen(QPen(Qt::white, 1.0));
        p.drawPath(arrow);

        p.restore();
    } else {
        // No GPS fix yet
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 0, 0, 100));
        p.drawRoundedRect(W / 2 - 110, H / 2 - 16, 220, 32, 6, 6);
        p.setPen(QColor(200, 200, 200));
        QFont f = p.font(); f.setPointSize(10); p.setFont(f);
        p.drawText(QRect(W / 2 - 110, H / 2 - 16, 220, 32),
                   Qt::AlignCenter, "Waiting for GPS fix…");
    }

    // --- HUD: zoom level + follow state ---
    QFont hf; hf.setPointSize(8);
    p.setFont(hf);
    p.setRenderHint(QPainter::Antialiasing, false);
    QString hud = QString("Zoom %1%2").arg(m_zoom).arg(m_follow ? "  ·  follow" : "");
    QRect hudRect(8, 8, 130, 18);
    p.fillRect(hudRect.adjusted(-2, -1, 2, 1), QColor(0, 0, 0, 140));
    p.setPen(Qt::white);
    p.drawText(hudRect, Qt::AlignVCenter | Qt::AlignLeft, hud);

    // Attribution (OSM requires it)
    QFont af; af.setPointSize(7);
    p.setFont(af);
    QString attr = "© OpenStreetMap contributors";
    QRect attrRect(0, H - 16, W, 16);
    p.fillRect(attrRect, QColor(0, 0, 0, 120));
    p.setPen(QColor(200, 200, 200));
    p.drawText(attrRect.adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignRight, attr);
}

// ---------------------------------------------------------------------------
// Zoom & pan
// ---------------------------------------------------------------------------
void MapWidget::applyZoom(int newZoom, QPointF anchorWidget) {
    newZoom = std::clamp(newZoom, MIN_ZOOM, MAX_ZOOM);
    if (newZoom == m_zoom) return;

    // World coords of the anchor point before zoom
    double ax = m_centerPixX + (anchorWidget.x() - width()  / 2.0);
    double ay = m_centerPixY + (anchorWidget.y() - height() / 2.0);

    // Scale world coordinates to new zoom level and keep anchor fixed
    double scale = std::pow(2.0, newZoom - m_zoom);
    m_centerPixX = ax * scale - (anchorWidget.x() - width()  / 2.0);
    m_centerPixY = ay * scale - (anchorWidget.y() - height() / 2.0);
    m_zoom = newZoom;

    // Cancel stale in-flight requests so their inflight slots are freed immediately
    // for the new zoom level — this is the main fix for the black-tile bug.
    cancelPendingRequests();

    // Re-center if follow mode is active
    if (m_follow && m_hasFix) centerOnDrone();

    update();
}

void MapWidget::wheelEvent(QWheelEvent* e) {
    int delta = e->angleDelta().y() > 0 ? 1 : -1;
    applyZoom(m_zoom + delta, e->position());
}

void MapWidget::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        m_dragging    = true;
        m_follow      = false; // user took manual control
        m_dragOrigin  = e->pos();
        m_dragCenterX = m_centerPixX;
        m_dragCenterY = m_centerPixY;
        setCursor(Qt::ClosedHandCursor);
    }
}

void MapWidget::mouseMoveEvent(QMouseEvent* e) {
    if (m_dragging) {
        QPoint delta = e->pos() - m_dragOrigin;
        m_centerPixX = m_dragCenterX - delta.x();
        m_centerPixY = m_dragCenterY - delta.y();
        update();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void MapWidget::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    repositionOverlay();
}

void MapWidget::repositionOverlay() {
    // Stack controls in the top-right corner
    int r = width() - 8;
    m_followBtn ->move(r - m_followBtn->width(),  8);
    m_zoomInBtn ->move(r - m_zoomInBtn->width() - m_zoomOutBtn->width() - 4, 44);
    m_zoomOutBtn->move(r - m_zoomOutBtn->width(), 44);
    m_trailCheck->move(r - m_trailCheck->width(), 80);
    m_followBtn ->raise();
    m_zoomInBtn ->raise();
    m_zoomOutBtn->raise();
    m_trailCheck->raise();
}

// ---------------------------------------------------------------------------
// Tile fetching
// ---------------------------------------------------------------------------
QString MapWidget::tileKey(int z, int x, int y) const {
    return QString("%1/%2/%3").arg(z).arg(x).arg(y);
}

void MapWidget::cancelPendingRequests() {
    // Clear the map first so that the finished() lambdas (which may fire
    // synchronously on some platforms when abort() is called) find an empty
    // map and do nothing harmful.
    QList<QNetworkReply*> replies = m_pendingReplies.values();
    m_pendingReplies.clear();
    for (auto* reply : replies)
        reply->abort();
}

void MapWidget::fetchTile(int z, int x, int y) {
    QString key = tileKey(z, x, y);
    if (m_tileCache.contains(key) || m_pendingReplies.contains(key)) return;

    // Throttle concurrent requests to avoid overwhelming the tile server
    if (m_pendingReplies.size() >= MAX_INFLIGHT) return;

    QUrl url(QString("https://tile.openstreetmap.org/%1/%2/%3.png").arg(z).arg(x).arg(y));
    QNetworkRequest req(url);
    // OSM tile usage policy requires a meaningful User-Agent header
    req.setHeader(QNetworkRequest::UserAgentHeader, "GCS/1.0 (drone-monitoring; educational)");
    req.setAttribute(QNetworkRequest::CacheLoadControlAttribute,
                     QNetworkRequest::PreferCache);

    auto* reply = m_nam->get(req);
    m_pendingReplies[key] = reply;

    connect(reply, &QNetworkReply::finished, this, [this, reply, key]() {
        m_pendingReplies.remove(key); // no-op if already cleared by cancelPendingRequests()
        if (reply->error() == QNetworkReply::NoError) {
            QPixmap pix;
            if (pix.loadFromData(reply->readAll()) && !pix.isNull()) {
                // Always cache the tile — the key encodes zoom/x/y so there is no
                // ambiguity, and caching tiles from nearby zoom levels means they
                // are available instantly when the user zooms back.
                if (m_tileCache.size() >= MAX_CACHED)
                    m_tileCache.erase(m_tileCache.begin());
                m_tileCache[key] = pix;
                update();
            }
        }
        reply->deleteLater();
    });
}
