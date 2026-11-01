# docs/evidence · 运行证据索引

> 这里只放**可复核的最小证据**。全量日志在 `log/` 下，但该目录被 `.gitignore` 忽略，不会随仓库迁移。
> 证据对应：Stage 5 环境审计（2026-10-26，`log/stage5_audit_f611c7zb/`）。

| 文件 | 来源 | 证明什么 |
|---|---|---|
| `stage5/test_results.log` | `log/stage5_audit_f611c7zb/test_results.log` | 26 tests, 0 errors, 0 failures, 0 skipped |
| `stage5/cleanup.json` | `log/stage5_audit_f611c7zb/cleanup.json` | domain 86 无残留节点 |
| `stage5/fake_execute_result.json` | `log/stage5_audit_f611c7zb/retry/fake_execute_result.json` | 右臂 Fake 演示完整执行结果 |
| `stage5/move_group_sigsegv.txt` | `log/stage5_audit_f611c7zb/retry/fake_execute.log` 崩溃段 | 退出清理 SIGSEGV 的调用栈 |

## 未入库的大文件

`retry/fake_execute_scenes.json` 约 2.8 MB，记录执行期间规划场景采样，未入库。

- 原始路径：`log/stage5_audit_f611c7zb/retry/fake_execute_scenes.json`
- SHA256：`40d39cd8645b04bb6a85e528757ba543916058983266b775d3e60efa5e4963b8`

若该日志目录丢失，此哈希只能证明"当时的文件是这个"，不能恢复内容。需要长期保留时应另行归档。

## 与验收记录的关系

按 `~/agent-prompts/agent.md` §10：标 `已完成` 的状态必须能指向这里的某份证据。
当前 Stage 5.1 的验收依据即本目录 + `docs/05_environment_audit.md`。
