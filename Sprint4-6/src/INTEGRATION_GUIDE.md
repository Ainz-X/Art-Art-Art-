# Analytics Module — 设计说明 & 接入指南

---

## 1. 模块设计说明

### 1.1 职责划分

| 维度 | 说明 |
|---|---|
| **本模块负责** | 记录运营事件（触发、保存、失败等）；管理任务生命周期计时；维护每日汇总计数；持久化到本地文件（jsonl + json） |
| **本模块不负责** | 摄像头采集、YOLO 推理、MediaPipe 手势检测、轮廓美化、SVG 导出、UI 渲染、网络通信 |
| **连接方式** | main.py 在关键分支处调用 `tracker.record_event()` / `tracker.start_job()` / `tracker.finish_job()` 等方法，单向调用，零回调 |

### 1.2 核心设计原则

1. **故障隔离**：所有内部方法用 `try/except` 包裹，`print()` 警告后继续，绝不向外抛异常
2. **线程安全**：内部用 `threading.Lock` 保护共享状态，支持多线程环境
3. **零依赖**：仅使用 Python 标准库（`json`, `pathlib`, `datetime`, `threading`, `uuid`, `time`）
4. **Append-only 日志**：`events.jsonl` 每行一个 JSON 对象，适合大文件流式读取和日后用 `pandas` 分析
5. **可观测**：`get_today_summary()` 随时返回当日摘要，方便 HUD 叠加或终端打印

### 1.3 数据结构

**事件日志**（`analytics/events.jsonl`）— 每行一条：

```json
{
  "timestamp": "2025-06-15T14:32:01.123",
  "event_type": "save_success",
  "job_id": "a1b2c3d4e5f6",
  "mode": "auto",
  "detail": "output/silhouette_042.svg",
  "extra": {"duration_ms": 1523.4}
}
```

**每日汇总**（`analytics/daily_summary.json`）：

```json
{
  "2025-06-15": {
    "trigger_count": 47,
    "vsign_count": 52,
    "auto_save": 38,
    "manual_save": 9,
    "save_success": 42,
    "save_fail": 5,
    "no_person": 3,
    "detection_fail": 1,
    "total_duration_ms": 63012.5,
    "duration_count": 42,
    "failure_reasons": {"no_person_detected": 3, "contour_too_small": 2},
    "multi_person_count": 0,
    "mode_usage": {}
  }
}
```

---

## 2. analytics_module.py

> 完整源码已作为独立文件交付（见 `analytics_module.py`）。
> 下面是公开 API 速查表：

| 方法 | 用途 | 调用时机 |
|---|---|---|
| `record_event(event_type, *, detail, mode, job_id, extra)` | 记录一条事件 | 任何关键节点 |
| `start_job(mode="auto")` → `str` | 开始一个保存任务，返回 job_id | 进入保存流程时 |
| `finish_job(job_id, detail="")` → `float\|None` | 任务成功，返回耗时 ms | 保存成功后 |
| `fail_job(job_id, reason="")` → `float\|None` | 任务失败，返回耗时 ms | 保存失败后 |
| `get_today_summary()` → `dict` | 获取今日统计摘要 | HUD 显示、终端打印 |
| `shutdown()` | 写入缓冲、保存汇总 | 程序退出前 |

**EventType 常量**（避免裸字符串拼写错误）：

```
SYSTEM_START / SYSTEM_SHUTDOWN
VSIGN_DETECTED / POSE_WINDOW_ENTER / COUNTDOWN_START / CAPTURE_FRAME
SAVE_AUTO_START / SAVE_MANUAL_START / SAVE_SUCCESS / SAVE_FAIL
NO_PERSON / DETECTION_FAIL
MODE_SWITCH / MULTI_PERSON / STYLE_APPLIED  （扩展预留）
```

---

## 3. main.py 最小改动示例

> **原则：不重写 main.py，只在关键 `if` 分支里添加一行调用。**

### 3.1 文件顶部 — 导入

```python
# ===== 在 main.py 顶部的 import 区域添加 =====
from analytics_module import AnalyticsTracker, EventType
```

### 3.2 main() 函数入口 — 初始化

```python
def main():
    # ===== 在现有初始化代码之后，主循环之前添加 =====
    tracker = AnalyticsTracker()  # 默认存储到 ./analytics/
    tracker.record_event(EventType.SYSTEM_START)
    current_job_id = None  # 用于跟踪当前保存任务

    # ... 原有初始化代码 (摄像头, YOLO, MediaPipe 等) ...
```

### 3.3 主循环内 — 各关键节点

以下每段代码块都是**在你现有 `if` 分支内额外插入的一行**，不修改原有逻辑：

```python
    # ───────── V-sign 检测成功 ─────────
    # 原有代码类似：if v_sign_detected:
    if v_sign_detected:
        tracker.record_event(EventType.VSIGN_DETECTED)  # ← 新增
        # ... 原有后续逻辑 ...

    # ───────── 进入 pose window ─────────
    # 原有代码类似：if enter_pose_window:
    if enter_pose_window:
        tracker.record_event(EventType.POSE_WINDOW_ENTER)  # ← 新增
        # ... 原有后续逻辑 ...

    # ───────── 倒计时开始（如果有） ─────────
    # 原有代码类似：if countdown_started:
    if countdown_started:
        tracker.record_event(EventType.COUNTDOWN_START)  # ← 新增

    # ───────── 没有检测到人 ─────────
    # 原有代码类似：if no_person_in_frame:
    if no_person_in_frame:
        tracker.record_event(EventType.NO_PERSON)  # ← 新增
```

### 3.4 自动保存流程

```python
    # ───────── 自动保存开始 ─────────
    # 原有代码类似：if should_auto_save:
    if should_auto_save:
        current_job_id = tracker.start_job(mode="auto")  # ← 新增
        try:
            # ... 原有保存逻辑：beautify_contour(), save_svg() 等 ...
            svg_path = save_svg(contour, filename)

            tracker.finish_job(current_job_id, detail=svg_path)  # ← 新增
        except Exception as e:
            tracker.fail_job(current_job_id, reason=str(e))  # ← 新增
        finally:
            current_job_id = None
```

### 3.5 手动保存流程（按键 S）

```python
    # ───────── 手动保存（按 S 键） ─────────
    # 原有代码类似：if key == ord('s'):
    if key == ord('s'):
        current_job_id = tracker.start_job(mode="manual")  # ← 新增
        try:
            # ... 原有保存逻辑 ...
            svg_path = save_svg(contour, filename)

            tracker.finish_job(current_job_id, detail=svg_path)  # ← 新增
        except Exception as e:
            tracker.fail_job(current_job_id, reason=str(e))  # ← 新增
        finally:
            current_job_id = None
```

### 3.6 程序退出前

```python
    # ===== 在主循环退出后、程序结束前添加 =====
    # 原有代码类似：cap.release(); cv2.destroyAllWindows()
    cap.release()
    cv2.destroyAllWindows()

    # 打印今日摘要并安全关闭
    summary = tracker.get_today_summary()
    print(f"\n===== 今日运营统计 =====")
    print(f"  V-sign 识别: {summary['vsign_count']} 次")
    print(f"  触发保存:    {summary['trigger_count']} 次")
    print(f"  自动保存:    {summary['auto_save']} 次")
    print(f"  手动保存:    {summary['manual_save']} 次")
    print(f"  保存成功:    {summary['save_success']} 次")
    print(f"  保存失败:    {summary['save_fail']} 次")
    print(f"  未检测到人:  {summary['no_person']} 次")
    print(f"  平均耗时:    {summary['avg_duration_ms']:.0f} ms")
    print(f"========================\n")
    tracker.shutdown()  # ← 关键：写入所有缓冲数据
```

### 3.7 可选 — HUD 上显示实时统计

```python
    # 在 OpenCV HUD 绘制循环中，可选添加：
    s = tracker.get_today_summary()
    hud_text = f"Today: {s['save_success']} ok / {s['save_fail']} fail"
    cv2.putText(frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 1, cv2.LINE_AA)
```

---

## 4. 事件格式规范

### 4.1 统一事件结构

每条事件都遵循以下 JSON 结构：

```json
{
  "timestamp": "ISO 8601，精确到毫秒",
  "event_type": "EventType 枚举值",
  "job_id": "关联的任务 ID（无关则为空字符串）",
  "mode": "auto / manual / multi / style / 空字符串",
  "detail": "具体描述（文件路径、错误信息等）",
  "extra": {"可扩展字典": "任意键值对"}
}
```

### 4.2 各事件类型的字段填写规范

| event_type | job_id | mode | detail | extra |
|---|---|---|---|---|
| `system_start` | — | — | — | — |
| `system_shutdown` | — | — | — | — |
| `vsign_detected` | — | — | 可选：置信度 | — |
| `pose_window_enter` | — | — | — | — |
| `countdown_start` | — | — | — | — |
| `capture_frame` | — | — | 帧文件路径 | — |
| `save_auto_start` | ✅ 自动填入 | `"auto"` | — | — |
| `save_manual_start` | ✅ 自动填入 | `"manual"` | — | — |
| `save_success` | ✅ 自动填入 | — | SVG 文件路径 | `{"duration_ms": 1234.5}` |
| `save_fail` | ✅ 自动填入 | — | 失败原因 | `{"duration_ms": 567.8}` |
| `no_person_detected` | — | — | — | `{"person_count": 0}` |
| `detection_fail` | — | — | 错误描述 | — |
| `mode_switch` | — | 新模式 | — | `{"from": "single", "to": "multi"}` |
| `multi_person_detected` | — | — | — | `{"person_count": 3}` |
| `style_applied` | — | `"style"` | 风格名称 | — |

### 4.3 失败原因命名规范

建议使用 `snake_case` 短语，保持统一：

- `no_person_detected` — 画面中没有检测到人
- `contour_too_small` — 轮廓面积太小，不足以生成有意义的 SVG
- `blur_detected` — 画面模糊
- `svg_write_error` — SVG 文件写入失败
- `camera_error` — 摄像头读取失败
- `timeout` — 处理超时

---

## 5. 后续扩展建议

### 5.1 短期扩展（代码层面已预留）

| 功能 | 实现方式 |
|---|---|
| **人数统计** | `record_event(EventType.MULTI_PERSON, extra={"person_count": n})` — extra 字段已支持 |
| **模式使用统计** | `record_event(EventType.MODE_SWITCH, mode="multi", extra={"from": "single", "to": "multi"})` — 在 `_empty_day()` 的 `mode_usage` 字典中累计 |
| **平均等待时长** | 在 `start_job` 之前再记录一个 `QUEUE_START` 事件，用两个时间戳之差计算等待时长 |
| **风格化模式** | `record_event(EventType.STYLE_APPLIED, mode="style", detail="sketch")` |

### 5.2 中期扩展 — Operator Dashboard

由于 `daily_summary.json` 已经是结构化 JSON，接入一个简单的 web 可视化只需要：

```python
# dashboard.py — 极简 Flask 示例（独立文件，不侵入 main.py）
from flask import Flask, jsonify
from analytics_module import AnalyticsTracker

app = Flask(__name__)
tracker = AnalyticsTracker()  # 读取同一个 analytics/ 目录

@app.route("/api/today")
def today():
    return jsonify(tracker.get_today_summary())

@app.route("/api/history")
def history():
    return jsonify(tracker.get_all_summaries())

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
```

前端用任意图表库（ECharts / Chart.js）读取这两个 JSON endpoint 即可。

### 5.3 长期扩展 — 更换存储后端

当前架构将持久化逻辑集中在 `_safe_flush_buffer()` 和 `_safe_save_summary()` 两个方法中。日后如果需要切换到 SQLite 或其他数据库，只需替换这两个方法的实现，上层 API 完全不变。

建议的演进路径：

```
jsonl + json 文件  →  SQLite（单文件数据库）→  PostgreSQL（多节点部署）
     ↑ 当前                ↑ 中期                    ↑ 远期
```

### 5.4 文件结构建议

```
项目根目录/
├── main.py                    # 主程序（最小改动）
├── analytics_module.py        # 统计模块（本文件）
├── analytics/                 # 自动创建的数据目录
│   ├── events.jsonl           # 事件流日志（append-only）
│   └── daily_summary.json     # 每日汇总
├── dashboard.py               # 可选：运营看板
└── ...其他现有文件...
```
