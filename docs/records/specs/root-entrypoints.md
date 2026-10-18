# 规格与计划：工作区根目录的「人的入口」

> autopilot 运行 `root-entrypoints`（2026-09-26）。用户原话：
> 「优化人工对于软件的访问与测试，当有不熟悉本项目的人进入工作区根目录后，
> 应当直接能点到主软件的启动入口和一些协助测试软件的入口（其实体放在你任何合理的位置即可，这里只要求入口）」

## 1. 问题（已实测确认）

工作区根目录当前只有：`requirements/ build/ CMakeLists.txt config/ docs/ scripts/ src/ tests/ tools/`。
**既无 README，也无任何启动脚本。** 一个不熟悉本项目的人站在根目录：

- 不知道哪个是主程序（`build/app` 藏在被 gitignore 的构建目录里）；
- 不知道 `scripts/manual_test_wizard.sh` 是人工测试入口；
- 不知道文档在 `docs/` 里各自是什么、哪些还没写。

## 2. 用户已确认的决策（Phase 0 澄清结果）

| 决策点 | 选定 |
|---|---|
| 入口形式 | **根目录可执行脚本 + README**（不要 .desktop） |
| 构建处理 | **自动检测并构建**（缺 `build/app` 时自动 cmake 配置 + 编译） |
| 入口清单 | **主软件（GUI）** + **人工测试向导（8 关）** + **文档导航** |
| 用户补充 | 文档导航需**预先标出"待完成"项**（课程设计报告等），且**这些内容日后创建时要同步更新进导航** |

## 3. 交付物（文件级边界）

| # | 文件 | 性质 | 验证命令 |
|---|---|---|---|
| 1 | `README.md`（根） | 门面：三条入口 + 环境要求 + 快速开始 | 人工通读；`grep` 断言三条入口均在 |
| 2 | `文档导航.md`（根） | 全部文档索引 + **待完成清单** + 维护约定 | `grep '待完成'` 命中报告/流程图/UML/用例表 |
| 3 | `启动软件.sh`（根） | 入口：自动构建 → 启动 GUI | `bash -n`；无显示环境下给出可操作提示而非卡死 |
| 4 | `人工测试向导.sh`（根） | 入口：自动构建 → 转交 `scripts/manual_test_wizard.sh` | `bash -n`；非 TTY 下转交后由向导自身拒绝并 exit 2 |
| 5 | `scripts/ensure_build.sh` | 共享「确保已构建」逻辑（被 3/4 source） | `bash -n`；删 build 后冷启动成功 |

**实体位置说明**：按用户"实体放在任何合理位置"的授权，真正的构建/启动逻辑落在 `scripts/ensure_build.sh`，
根目录只放**入口**。

## 4. 必须处理的真实故障模式（侦察所得）

1. **Qt6 缺失时 CMake 静默跳过 app 目标**（`CMakeLists.txt:83,129`）→ 脚本必须检测
   `build/app` 是否存在，并给出「装 qt6-base-devel / 改用 CLI 方式」的可操作提示，而不是报一句"启动失败"。
2. **工作区被移动后 `CMakeCache.txt` 记的仍是旧路径** → `cmake --build` 报
   `CMakeCache.txt directory ... is different than the directory ... was created`。
   本轮开局我就踩了这个。→ `ensure_build` 必须比对缓存的 `CMAKE_HOME_DIRECTORY`，不符则重建构建目录。
3. **无显示环境**（DISPLAY 与 WAYLAND_DISPLAY 均为空）→ 启动 GUI 会崩。
   → 先检测，给出 `--plan-summary` / `QT_QPA_PLATFORM=offscreen --render-window` 两条替代路径。
4. **向导需要 TTY**（`manual_test_wizard.sh:601` 已有 `[[ ! -t 0 ]]` 保护）→ 入口脚本不得吞掉这条保护，
   也不得在非 TTY 下"假装跑通"。

## 5. 不做（明确排除）

- 不建 `.desktop` 快捷方式（用户未选；且依赖绝对路径，换目录即失效）。
- 不为 `ctest`、CLI 探针建**独立根入口脚本**（用户在多选中未勾选）。
  它们只在 README 的「附：其他常用命令」里作为命令列出，不占入口位。
- 不改动 `build/` 的 gitignore 状态（构建产物不入库）。

## 6. 验收判据

1. 根目录 `ls` 一眼可见入口：`README.md`、`启动软件.sh`、`AGENTS.md`、`CMakeLists.txt`；用户侧测试入口 `人工测试向导.sh` 在 `start-here/run/`，文档索引 `文档导航.md` 在 `docs/`。
2. **冷启动可复现**：`rm -rf build && ./启动软件.sh` 能自动配置+编译并启动（有显示时）。
3. 故障模式 1/2/3 各自给出可操作提示，且退出码非 0（不静默）。
4. `bash -n` 全部通过。
5. **回归**：`ctest` 仍全绿（本批新增 `doc_links_lint` 后为 **18 个目标**；基线 `c9ed77d` 为 17）；三策略黄金值逐位不变。
6. 文档导航中「待完成」项被显式标注，且写明"新增交付物必须同步更新本表"。

## 7. 执行计划（按依赖排序）

1. 写 `scripts/ensure_build.sh`（被依赖，先做）。
2. 写 `启动软件.sh`（根目录）、`人工测试向导.sh`（`start-here/run/`，依赖 1）。
3. 写 `README.md`、`文档导航.md`（内容取决于 1–2 的实际用法）。
4. QA：`bash -n` 全部；冷启动演练；故障模式演练；`ctest` 回归。
5. Validation：verifier（对照本规格逐条验）+ code-reviewer（脚本健壮性）并行，双通过才算完成。
