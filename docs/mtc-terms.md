# MTC 术语速查

> **按需查阅，不属于常驻提示词。** 首次接触 MTC 概念时读这份。
> 讲解时第一次出现术语仍须当场给中文解释（规则见 `~/agent-prompts/AGENTS.md` §5.8）。
> 除标注「已验证」的条目外，下表是对 MTC 行为的概括，属**合理推断**，未逐条对照 MTC 0.1.5 源码。

| 术语 | 一句话解释 |
|---|---|
| `SerialContainer` | 串联容器：前一步的输出就是后一步的输入，按顺序走 |
| `Alternatives` | 并联容器：同一目标列多个候选各算一遍，任意一条能接下去就算成功，其余剪掉（工作模型，待源码核实） |
| Generator（`GenerateGraspPose` / `GeneratePlacePose`） | 生成器：从无到有造出目标位姿候选 |
| `ComputeIK` | 把末端目标位姿换算成关节角；同一个位姿可能有多个解 |
| `Connect` | 把两段已知状态用一条无碰撞路径接起来，中间怎么走交给规划器 |
| `MoveTo` / `MoveRelative` | 前者到某个关节目标，后者相对当前位置移动一段距离 |
| Propagator | 从下游需求反推上游该处于什么状态（工作模型，待源码核实） |
| ACM（Allowed Collision Matrix） | 允许碰撞对列表：决定哪些几何体之间不检查碰撞 |
| attached object | 挂在机器人某个 link 上的物体，之后规划时它跟着动 |
| Stage state | 每个阶段结束后的场景与机器人状态，是下游阶段的输入 |

日志里 `solutions=N` 的含义随 Stage 类型变化：在 Generator / ComputeIK 上是该阶段产出的候选数，
在 Alternatives 上是**其所有分支产出之和**（已验证：323 = 60 个分支解数之和），只有 `Final: solutions` 才是整条任务的解数。
