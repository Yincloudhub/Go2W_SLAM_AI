# `_RECOVERY_HISTORY` NameError 及模块级变量陷阱

> 2026-06-16 | 提交 4c05dea

## 现象

```
post-navigation recovery failed: name '_RECOVERY_HISTORY' is not defined
```

机器人导航完成后触发 recovery 逻辑时崩溃。

## 根因

`scripts/run_robot_closed_loop.py:1148` 使用了 `_RECOVERY_HISTORY` 但**整个文件从未定义它**：

```python
novelty = 0.3 if direction not in (_RECOVERY_HISTORY or []) else 0.0
```

`or []` 回退说明作者意图是"可能为 None，此时用空列表"。但变量本身不存在 → NameError。

## 为什么之前没触发

之前的导航在更早阶段就被拦截（`left_obstacle_confidence_too_low`），从未走到 recovery 逻辑。修复 confidence 检查后导航执行成功，recovery 才被触发，暴露了这个 latent bug。

## 修复

在文件顶部（imports 之后）添加模块级定义：

```python
# Track recent recovery directions to avoid oscillation (module-level so it
# persists across recovery attempts within the same session).
_RECOVERY_HISTORY: list[str] = []
```

## 一般模式

Python 模块级变量"使用但未定义"是 silent bug——代码合入时不会报错，只在特定执行路径触发时崩溃。排查方法：

```bash
grep -n 'VARIABLE_NAME' script.py
# 如果只有 usage 没有 definition → NameError waiting to happen
```
