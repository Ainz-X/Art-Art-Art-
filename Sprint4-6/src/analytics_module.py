"""
analytics_module.py — 运营统计模块
====================================
职责：记录系统运行过程中的关键运营事件，方便后续统计分析。
不负责：摄像头控制、YOLO/MediaPipe 推理、轮廓算法、SVG 导出、UI 渲染。

设计原则：
  1. 低耦合 —— main.py 仅在关键节点调用本模块的公开方法
  2. 故障隔离 —— 本模块的任何异常都会被内部捕获，绝不会影响主流程
  3. 零第三方依赖 —— 仅使用 Python 标准库（json, csv, datetime, pathlib, threading…）
  4. 易于扩展 —— 预留 extra 字段与事件类型枚举，方便日后接入 web dashboard

持久化方式：
  - 事件日志  →  analytics/events.jsonl  （每行一个 JSON 对象，append-only）
  - 每日汇总  →  analytics/daily_summary.json  （每天一个 key，覆盖写入）

用法示例（最小接入）：
    from analytics_module import AnalyticsTracker
    tracker = AnalyticsTracker()

    # 检测到 V-sign
    tracker.record_event("vsign_detected")

    # 开始一个保存任务
    job_id = tracker.start_job(mode="auto")
    # … 保存成功 …
    tracker.finish_job(job_id, detail="saved to output/xxx.svg")
    # … 或保存失败 …
    tracker.fail_job(job_id, reason="no_person_detected")

    # 获取今日摘要
    summary = tracker.get_today_summary()

    # 程序退出前
    tracker.shutdown()
"""

from __future__ import annotations

import json
import time
import uuid
import threading
from datetime import datetime, date
from pathlib import Path
from typing import Any, Optional


# ═══════════════════════════════════════════════════════════
#  事件类型常量（方便 main.py 使用，避免裸字符串）
# ═══════════════════════════════════════════════════════════

class EventType:
    """统一事件类型枚举，按字符串常量管理，避免拼写错误。"""
    SYSTEM_START       = "system_start"
    SYSTEM_SHUTDOWN    = "system_shutdown"

    VSIGN_DETECTED     = "vsign_detected"        # V-sign 识别成功
    POSE_WINDOW_ENTER  = "pose_window_enter"      # 进入 pose window
    COUNTDOWN_START    = "countdown_start"        # 倒计时开始
    CAPTURE_FRAME      = "capture_frame"          # 抓取了一帧

    SAVE_AUTO_START    = "save_auto_start"        # 自动保存开始
    SAVE_MANUAL_START  = "save_manual_start"      # 手动保存开始
    SAVE_SUCCESS       = "save_success"           # 保存成功
    SAVE_FAIL          = "save_fail"              # 保存失败

    NO_PERSON          = "no_person_detected"     # 画面中没检测到人
    DETECTION_FAIL     = "detection_fail"         # 检测/分割失败（通用）

    # ---- 扩展预留 ----
    MODE_SWITCH        = "mode_switch"            # 模式切换（单人/多人/风格化）
    MULTI_PERSON       = "multi_person_detected"  # 多人检测到
    STYLE_APPLIED      = "style_applied"          # 风格化效果应用


# ═══════════════════════════════════════════════════════════
#  核心 Tracker 类
# ═══════════════════════════════════════════════════════════

class AnalyticsTracker:
    """
    运营统计追踪器。线程安全，内部异常不会外泄。

    Parameters
    ----------
    analytics_dir : str | Path
        事件日志和每日汇总文件的存储目录，默认 "analytics"。
    auto_flush : bool
        是否每条事件都立即写盘，默认 True（安全优先）。
        如果在高频场景下觉得 IO 太多，可以设为 False，然后定期 flush()。
    """

    def __init__(
        self,
        analytics_dir: str | Path = "analytics",
        auto_flush: bool = True,
    ):
        self._dir = Path(analytics_dir)
        self._events_file = self._dir / "events.jsonl"
        self._summary_file = self._dir / "daily_summary.json"
        self._auto_flush = auto_flush

        self._lock = threading.Lock()        # 保护所有共享状态
        self._active_jobs: dict[str, dict] = {}   # job_id → {start_time, mode, ...}
        self._buffer: list[dict] = []        # 尚未写盘的事件
        self._daily_summary: dict = {}       # {"2025-03-30": {counters...}, ...}

        # 初始化目录和文件
        self._safe_init()

    # ─── 初始化 ────────────────────────────────────────────

    def _safe_init(self):
        """安全创建目录、加载已有汇总。"""
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
            if self._summary_file.exists():
                with open(self._summary_file, "r", encoding="utf-8") as f:
                    self._daily_summary = json.load(f)
        except Exception as e:
            # 即使初始化失败，也不影响主程序
            print(f"[AnalyticsTracker] 初始化警告: {e}")
            self._daily_summary = {}

    # ─── 公开 API：事件记录 ────────────────────────────────

    def record_event(
        self,
        event_type: str,
        *,
        detail: str = "",
        mode: str = "",
        job_id: str = "",
        extra: dict[str, Any] | None = None,
    ) -> dict:
        """
        记录一条运营事件。

        Parameters
        ----------
        event_type : str
            事件类型，建议使用 EventType 中的常量。
        detail : str
            事件详情，例如文件路径、错误信息。
        mode : str
            当前模式（"auto" / "manual" / "multi" / "style" / ""）。
        job_id : str
            关联的任务 ID（如果与某次保存任务相关）。
        extra : dict, optional
            附加字段，用于未来扩展（人数、风格名等）。

        Returns
        -------
        dict
            写入的事件字典（含 timestamp）。
        """
        event = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event_type": event_type,
            "job_id": job_id,
            "mode": mode,
            "detail": detail,
            "extra": extra or {},
        }
        self._safe_append(event)
        self._safe_update_counter(event_type)
        return event

    # ─── 公开 API：任务（Job）生命周期 ────────────────────

    def start_job(self, mode: str = "auto") -> str:
        """
        开始一个保存任务，返回唯一 job_id。

        在 main.py 中，当系统进入"准备保存"阶段时调用。
        """
        job_id = uuid.uuid4().hex[:12]
        now = time.monotonic()
        with self._lock:
            self._active_jobs[job_id] = {
                "start_time": now,
                "mode": mode,
            }
        self.record_event(
            EventType.SAVE_AUTO_START if mode == "auto" else EventType.SAVE_MANUAL_START,
            job_id=job_id,
            mode=mode,
        )
        return job_id

    def finish_job(self, job_id: str, detail: str = "") -> Optional[float]:
        """
        标记任务成功完成，返回耗时（毫秒）。

        Parameters
        ----------
        job_id : str
            start_job() 返回的 ID。
        detail : str
            保存结果详情，例如 SVG 文件路径。

        Returns
        -------
        float | None
            从 start_job 到 finish_job 的耗时（ms），如果 job_id 无效则返回 None。
        """
        duration_ms = self._pop_job(job_id)
        self.record_event(
            EventType.SAVE_SUCCESS,
            job_id=job_id,
            detail=detail,
            extra={"duration_ms": round(duration_ms, 1)} if duration_ms is not None else {},
        )
        # 注意：record_event 内部已经通过 _COUNTER_MAP 更新了 save_success 计数
        if duration_ms is not None:
            self._safe_accumulate_duration(duration_ms)
        return duration_ms

    def fail_job(self, job_id: str, reason: str = "") -> Optional[float]:
        """
        标记任务失败，返回耗时（毫秒）。

        Parameters
        ----------
        job_id : str
            start_job() 返回的 ID。
        reason : str
            失败原因，例如 "no_person_detected"、"contour_too_small"。
        """
        duration_ms = self._pop_job(job_id)
        self.record_event(
            EventType.SAVE_FAIL,
            job_id=job_id,
            detail=reason,
            extra={"duration_ms": round(duration_ms, 1)} if duration_ms is not None else {},
        )
        # 注意：record_event 内部已经通过 _COUNTER_MAP 更新了 save_fail 计数
        self._safe_increment_failure_reason(reason)
        return duration_ms

    # ─── 公开 API：查询统计 ─────────────────────────────────

    def get_today_summary(self) -> dict:
        """
        获取今日的运营统计摘要。

        Returns
        -------
        dict
            包含 trigger_count, auto_save, manual_save, success, fail,
            no_person, vsign_count, avg_duration_ms, failure_reasons 等。
        """
        today_key = date.today().isoformat()
        with self._lock:
            data = self._daily_summary.get(today_key, self._empty_day())
        return {
            "date": today_key,
            **data,
            "avg_duration_ms": self._calc_avg_duration(data),
        }

    def get_all_summaries(self) -> dict:
        """返回所有天的汇总数据（用于未来 dashboard）。"""
        with self._lock:
            return dict(self._daily_summary)

    # ─── 公开 API：持久化 ──────────────────────────────────

    def flush(self):
        """手动将缓冲区事件写入 events.jsonl。"""
        self._safe_flush_buffer()

    def save_summary(self):
        """手动将每日汇总写入 daily_summary.json。"""
        self._safe_save_summary()

    def shutdown(self):
        """
        程序退出前调用：记录关闭事件，写入所有缓冲，保存汇总。
        """
        self.record_event(EventType.SYSTEM_SHUTDOWN)
        self._safe_flush_buffer()
        self._safe_save_summary()

    # ═══════════════════════════════════════════════════════
    #  内部实现（以下方法均为私有）
    # ═══════════════════════════════════════════════════════

    @staticmethod
    def _empty_day() -> dict:
        """返回一个空白的日统计结构。"""
        return {
            "trigger_count": 0,      # 总触发次数（进入 pose window）
            "vsign_count": 0,        # V-sign 识别成功次数
            "auto_save": 0,          # 自动保存次数
            "manual_save": 0,        # 手动保存次数
            "save_success": 0,       # 保存成功次数
            "save_fail": 0,          # 保存失败次数
            "no_person": 0,          # 没检测到人的次数
            "detection_fail": 0,     # 通用检测失败
            "total_duration_ms": 0.0,  # 累计保存耗时（ms）
            "duration_count": 0,     # 有耗时记录的任务数
            "failure_reasons": {},   # {reason_str: count}
            # ---- 扩展预留 ----
            "multi_person_count": 0,
            "mode_usage": {},        # {"single": N, "multi": N, "style": N}
        }

    def _today_data(self) -> dict:
        """获取或创建今天的汇总字典（必须在 _lock 内调用）。"""
        today_key = date.today().isoformat()
        if today_key not in self._daily_summary:
            self._daily_summary[today_key] = self._empty_day()
        return self._daily_summary[today_key]

    # ─── 事件写入 ──────────────────────────────────────────

    def _safe_append(self, event: dict):
        """将事件添加到缓冲区，根据策略决定是否立即写盘。"""
        try:
            with self._lock:
                self._buffer.append(event)
            if self._auto_flush:
                self._safe_flush_buffer()
        except Exception as e:
            print(f"[AnalyticsTracker] 事件写入警告: {e}")

    def _safe_flush_buffer(self):
        """将缓冲区中的事件 append 到 events.jsonl。"""
        try:
            with self._lock:
                if not self._buffer:
                    return
                events_to_write = list(self._buffer)
                self._buffer.clear()

            with open(self._events_file, "a", encoding="utf-8") as f:
                for ev in events_to_write:
                    f.write(json.dumps(ev, ensure_ascii=False) + "\n")
        except Exception as e:
            print(f"[AnalyticsTracker] flush 警告: {e}")

    def _safe_save_summary(self):
        """将每日汇总写入 daily_summary.json。"""
        try:
            with self._lock:
                data = dict(self._daily_summary)
            with open(self._summary_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[AnalyticsTracker] 汇总保存警告: {e}")

    # ─── 计数器更新 ────────────────────────────────────────

    # 事件类型 → 汇总字段名 的映射
    _COUNTER_MAP: dict[str, str] = {
        EventType.VSIGN_DETECTED:    "vsign_count",
        EventType.POSE_WINDOW_ENTER: "trigger_count",
        EventType.SAVE_AUTO_START:   "auto_save",
        EventType.SAVE_MANUAL_START: "manual_save",
        EventType.SAVE_SUCCESS:      "save_success",
        EventType.SAVE_FAIL:         "save_fail",
        EventType.NO_PERSON:         "no_person",
        EventType.DETECTION_FAIL:    "detection_fail",
        EventType.MULTI_PERSON:      "multi_person_count",
    }

    def _safe_update_counter(self, event_type: str):
        """根据事件类型增加对应的日统计计数器。"""
        try:
            field = self._COUNTER_MAP.get(event_type)
            if field:
                with self._lock:
                    self._today_data()[field] += 1
                # 注意：save 必须在 lock 外调用，避免死锁
                self._safe_save_summary()
        except Exception as e:
            print(f"[AnalyticsTracker] 计数更新警告: {e}")

    def _safe_accumulate_duration(self, duration_ms: float):
        """累积一次成功任务的耗时。"""
        try:
            with self._lock:
                day = self._today_data()
                day["total_duration_ms"] += duration_ms
                day["duration_count"] += 1
            self._safe_save_summary()
        except Exception as e:
            print(f"[AnalyticsTracker] 耗时累积警告: {e}")

    def _safe_increment_failure_reason(self, reason: str):
        """累积失败原因统计。"""
        if not reason:
            return
        try:
            with self._lock:
                reasons = self._today_data()["failure_reasons"]
                reasons[reason] = reasons.get(reason, 0) + 1
            self._safe_save_summary()
        except Exception as e:
            print(f"[AnalyticsTracker] 失败原因累积警告: {e}")

    # ─── Job 管理 ──────────────────────────────────────────

    def _pop_job(self, job_id: str) -> Optional[float]:
        """弹出一个活跃任务并返回耗时（ms），无效 ID 返回 None。"""
        with self._lock:
            job = self._active_jobs.pop(job_id, None)
        if job is None:
            return None
        return (time.monotonic() - job["start_time"]) * 1000.0

    # ─── 工具函数 ──────────────────────────────────────────

    @staticmethod
    def _calc_avg_duration(day_data: dict) -> float:
        """计算平均保存耗时（ms）。"""
        count = day_data.get("duration_count", 0)
        total = day_data.get("total_duration_ms", 0.0)
        return round(total / count, 1) if count > 0 else 0.0


# ═══════════════════════════════════════════════════════════
#  便捷全局实例（可选使用方式）
# ═══════════════════════════════════════════════════════════
#
# 如果不想在 main.py 里手动实例化，可以直接：
#   from analytics_module import tracker
#   tracker.record_event(...)
#
# 但推荐在 main.py 中显式创建实例，更灵活。

_default_tracker: Optional[AnalyticsTracker] = None


def get_tracker(analytics_dir: str = "analytics") -> AnalyticsTracker:
    """获取或创建全局默认 tracker（懒加载、线程安全）。"""
    global _default_tracker
    if _default_tracker is None:
        _default_tracker = AnalyticsTracker(analytics_dir=analytics_dir)
    return _default_tracker


# ═══════════════════════════════════════════════════════════
#  自测入口
# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=== AnalyticsTracker 自测 ===\n")

    t = AnalyticsTracker(analytics_dir="analytics_test")

    # 模拟系统启动
    t.record_event(EventType.SYSTEM_START)

    # 模拟一次完整的自动保存流程
    t.record_event(EventType.VSIGN_DETECTED)
    t.record_event(EventType.POSE_WINDOW_ENTER)
    job = t.start_job(mode="auto")
    time.sleep(0.15)  # 模拟处理时间
    t.finish_job(job, detail="output/silhouette_001.svg")

    # 模拟一次失败
    t.record_event(EventType.VSIGN_DETECTED)
    t.record_event(EventType.POSE_WINDOW_ENTER)
    job2 = t.start_job(mode="auto")
    time.sleep(0.05)
    t.fail_job(job2, reason="no_person_detected")

    # 模拟手动保存
    job3 = t.start_job(mode="manual")
    time.sleep(0.1)
    t.finish_job(job3, detail="output/silhouette_002.svg")

    # 模拟无人检测
    t.record_event(EventType.NO_PERSON)

    # 查看今日摘要
    summary = t.get_today_summary()
    print("今日摘要：")
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    # 关闭
    t.shutdown()

    # 验证文件
    print(f"\n事件日志文件: {t._events_file}")
    print(f"每日汇总文件: {t._summary_file}")
    if t._events_file.exists():
        lines = t._events_file.read_text(encoding="utf-8").strip().split("\n")
        print(f"共记录 {len(lines)} 条事件")
        print("最后一条:", lines[-1])

    print("\n=== 自测完成 ===")
