#pragma once
#include <QWidget>
#include <QPlainTextEdit>

// ---------------------------------------------------------------------------
// TerminalWidget — scrollable log output from PKT_LOG messages.
// Color-coded by level. Has a Clear button.
// ---------------------------------------------------------------------------

class TerminalWidget : public QWidget {
    Q_OBJECT
public:
    explicit TerminalWidget(QWidget* parent = nullptr);
    void appendMessage(uint8_t level, const QString& text);

private:
    QPlainTextEdit* m_view = nullptr;
};
