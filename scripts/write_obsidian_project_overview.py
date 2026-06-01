#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Update external Obsidian GO2W planning notes with the current engineering baseline."""

from __future__ import annotations

from pathlib import Path


START = "<!-- GO2W_2026_06_01_STATUS_START -->"
END = "<!-- GO2W_2026_06_01_STATUS_END -->"


PROJECT_BLOCK = f"""{START}

## 2026-06-01 工程状态同步：闭环够用基线与下一步

当前工程判断：主链路已经达到“够用可演示”，下一步应转向录点、巡检任务模板、日志评估和弱网实验，不继续把精力放在传感器侧车的极致资源压缩上。

已验证的主闭环：

```text
XT16 LiDAR
  -> Unitree SLAM / relocation
  -> C++ GatewayClient / OperatorPanel
  -> SemanticRouter / TaskQueue validator
  -> SafetyGate / QueueExecutor
  -> Web UI / LLM 输入口
```

当前可确认的工程事实：

- 趴卧静止调试下，UI 可读到 `loc=true`、`map=true`、`motion=false`、`safety=ok`。
- 中文 Web UI 已能显示机器狗回复、当前位置、视觉理解、安全策略、任务输入和视觉离线降级。
- DeepYOLO / D435I 已被定义为可选语义侧车：正常时提供低频语义摘要，stale 或相机离线时不阻塞 LiDAR + SLAM 主链路。
- XT16 LiDAR 和 Unitree SLAM 是当前安全主链路；双目、DeepYOLO、TI 雷达、语音、拍照都应作为可插拔增强项。
- `unitree_slam` 仍是最大 CPU 项，但属于厂商二进制；比赛前不建议直接改 `/unitree` 厂商参数。

近期推进顺序：

1. 站起后重新启动/检查 SLAM 与 XT16，执行重定位，确认 `loc=true`、`safety=ok`。
2. 录制和复核真实拓扑点，把趴卧标定点和可导航点分开管理。
3. 做巡检任务模板：多点队列、到点回复、失败停止、结束报告。
4. 把 `capture_keyframe` 从事件记录推进到真实拍照命令，但保持低频、可降级。
5. 补 `state_journal` 或长驻 operator core，保存任务队列、到达事件、SafetyGate 裁决和人工确认。
6. 先积累真实日志和失败样本，再考虑 MiniMind-GO2；没有 500-1000 条高质量样本前不把微调作为主线。
7. 弱网实验从概念改成指标：全量视频、关键帧+语义、纯语义、本地智能体闭环四组对比。
8. TI 雷达 / NX 先只输出 `RadarDetectionSummary`，用于异常告警、`slow/confirm/inspect_area`，不把原始 ADC 或高频点云送入 LLM。

下一阶段的核心卖点应表述为：

> 机器人在弱链路下依靠本地 SLAM、安全执行链路和边缘 LLM 完成可解释巡检；多模态传感器只提供低频语义摘要，任何感知异常都能降级而不拖垮主闭环。

{END}
"""


TRACK_BLOCK = f"""{START}

## 2026-06-01 赛道与研发路线更新

主赛道判断不变：仍建议主报“大模型与智能体系统”，弱网语义通信、感知辅助通信和多模态边缘节点作为支撑实验。现在工程上已经不只是概念规划，主闭环具备可运行基线：

```text
自然语言 / UI 输入
  -> 确定性拓扑匹配优先
  -> LLM 处理模糊目标和任务解释
  -> TaskQueue IR
  -> SafetyGate
  -> QueueExecutor / SLAM Gateway
  -> UI 中文反馈与运行日志
```

后续研发重点应从“再加功能”转为“让演示链路稳定、可解释、可量化”：

- 录点：站立状态下重新验证重定位和拓扑点，形成可巡检节点集合。
- 巡检：做 2-3 个点的任务模板，每个点有到达回复、拍照/语义事件、失败原因。
- UI：继续强化中文反馈屏，让评委能看到当前去哪、到了哪里、为什么停、视觉是否离线。
- 弱网：用真实指标证明语义通信有用，而不是只讲 6G 概念。
- 模型：Qwen3-4B / 本地 LLM 先作为主力；MiniMind-GO2 等轻量模型等日志足够后再做对比实验。
- 多模态：D435I、DeepYOLO、TI 雷达/NX 都是可选摘要源，不能成为主链路硬依赖。

当前要避免的偏差：

- 不把项目讲成 LLM 直接控制机器狗。
- 不承诺没有 SLAM 也能 30m 自主导航。
- 不把相机或雷达离线视为主系统失败；主链路应能降级为 LiDAR + SLAM。
- 不在比赛前冒险改厂商 SLAM 参数。
- 不为了模型微调拖慢现场闭环和数据采集。

建议下一阶段验收：

```text
1. 站立状态重定位成功。
2. 至少录入 3-5 个真实拓扑点。
3. UI 发起一条多点巡检 dry-run。
4. 至少一个真实点位完成低速到达验证。
5. 到达后有中文反馈、任务日志和关键帧/语义事件。
6. 弱网模式下 UI 展示保留语义、丢弃大流量数据的策略。
```

{END}
"""


TARGETS = {
    Path("F:/browser/go2w_project_deepening_plan.md"): PROJECT_BLOCK,
    Path("F:/browser/go2w_llm_agent_track_analysis.md"): TRACK_BLOCK,
}


def upsert_block(path: Path, block: str) -> None:
    if not path.exists():
        raise FileNotFoundError(path)
    text = path.read_text(encoding="utf-8")
    if START in text and END in text:
        before = text.split(START, 1)[0].rstrip()
        after = text.split(END, 1)[1].lstrip()
        next_text = before + "\n\n" + block.rstrip() + "\n\n" + after
    else:
        next_text = text.rstrip() + "\n\n---\n\n" + block.rstrip() + "\n"
    path.write_text(next_text, encoding="utf-8")


def main() -> int:
    for path, block in TARGETS.items():
        upsert_block(path, block)
        print(f"updated={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
