#include "TerminalWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QDateTime>
#include <QLabel>
#include <QScrollBar>

TerminalWidget::TerminalWidget(QWidget* parent) : QWidget(parent) {
    auto* vl = new QVBoxLayout(this);
    vl->setContentsMargins(4,4,4,4);
    vl->setSpacing(4);

    auto* header = new QHBoxLayout();
    auto* title  = new QLabel("TERMINAL", this);
    title->setStyleSheet("color: #aaa; font-weight: bold;");
    auto* clearBtn = new QPushButton("Clear", this);
    clearBtn->setMaximumWidth(70);
    clearBtn->setStyleSheet("QPushButton { background: #3a2222; color: #ccc; border: 1px solid #633; }"
                            "QPushButton:hover { background: #632222; }");
    header->addWidget(title);
    header->addStretch(1);
    header->addWidget(clearBtn);

    m_view = new QPlainTextEdit(this);
    m_view->setReadOnly(true);
    m_view->setMaximumBlockCount(2000);
    m_view->setStyleSheet("QPlainTextEdit { background: #0a0a0a; color: white; "
                          "font-family: monospace; font-size: 11px; border: 1px solid #333; }");

    connect(clearBtn, &QPushButton::clicked, m_view, &QPlainTextEdit::clear);

    vl->addLayout(header);
    vl->addWidget(m_view);
}

void TerminalWidget::appendMessage(uint8_t level, const QString& text) {
    static const char* levelStr[]  = { "DBG", "INF", "WRN", "ERR" };
    static const char* levelColor[]= { "#888888", "#ffffff", "#ffcc00", "#ff4444" };
    uint8_t lv = qMin<uint8_t>(level, 3);
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString html = QString("<span style='color:%1'>[%2] [%3] %4</span>")
                       .arg(levelColor[lv])
                       .arg(ts)
                       .arg(levelStr[lv])
                       .arg(text.toHtmlEscaped());
    m_view->appendHtml(html);
    // Auto-scroll to bottom
    auto* sb = m_view->verticalScrollBar();
    sb->setValue(sb->maximum());
}
