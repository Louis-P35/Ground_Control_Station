# Coding Style — Ground Control Station & MicroFlight ESP32

Applies to all C++ files in this repository (GCS and ESP32 firmware).

---

## Brace style — Allman

Opening braces are **always on a new line**, at the same indentation level as the
statement that opens the block. This applies to every construct without exception:
functions, classes, structs, `if`, `else`, `for`, `while`, `switch`, `do`, lambda,
`namespace`, and `try`/`catch`.

```cpp
// CORRECT
void foo(int x)
{
    if (x > 0)
    {
        for (int i = 0; i < x; ++i)
        {
            doSomething(i);
        }
    }
    else
    {
        doOther();
    }
}

// WRONG — opening brace on the same line
void foo(int x) {
    if (x > 0) {
    }
}
```

Single-line bodies **must** still use braces, and the body is **always on its own line**:

```cpp
// CORRECT
if (err)
{
    return false;
}

// WRONG — no braces
if (err) return false;

// WRONG — body on the same line as the brace
if (err) { return false; }
```

**Exception — inline methods in a class definition inside a `.h` file** only:

```cpp
// ALLOWED in .h class bodies
bool newAttitude() const { return m_newAttitude; }
```

This exception does **not** apply to `.cpp` files, nor to `if`/`for`/`while` bodies anywhere.

---

## Indentation

- **4 spaces** — no tabs.

---

## Naming

| Construct | Convention | Example |
|-----------|-----------|---------|
| Local variable | `camelCase` | `frameCount` |
| Member variable | `m_camelCase` | `m_rxBuf` |
| Static/global | `g_camelCase` | `g_udp` |
| Static local | `s_camelCase` | `s_lastMs` |
| Function / method | `camelCase` | `parseRxFrame()` |
| Class / struct | `PascalCase` | `SpiSlave`, `PktLog` |
| Enum class value | `PascalCase` | `SpiFrameType::Attitude` |
| Constant / macro | `UPPER_SNAKE` | `SPI_FRAME_SIZE` |
| Namespace | `lowercase` | `Config` |

---

## Miscellaneous

- Prefer `static_cast<>` over C-style casts.
- Use `nullptr`, not `NULL` or `0` for pointers.
- Always initialise structs with `{}` before filling fields (e.g. `PktLog pkt{};`).
- Prefer `const` / `constexpr` over `#define` for constants.
- One class per header/source pair; file name matches class name exactly.
