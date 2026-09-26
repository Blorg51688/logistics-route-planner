# AGENTS.md — 本工作区的 agent 操作手册

> 本文件**自动注入每个新会话**，所以只放"每次动手都需要知道"的东西，务必精简。
> 项目是什么、怎么跑 → [`README.md`](README.md)；文档地图与**待完成清单** → [`文档导航.md`](文档导航.md)（单一真相源，**不要在本文件重复维护**）。

## 1. 硬约束（课程要求，违反即扣分）

- **STL 口径**：课程只考"算法"，不考"轮子"。判据是 **手写「算法」，不手写「轮子」**：
  - ❌ **禁止把算法主体交给 STL**：`std::sort` / `std::partial_sort` / `std::nth_element` /
    `std::min_element` / `std::max_element` / `std::lower_bound` / `std::upper_bound` /
    `std::priority_queue` / `std::make_heap`·`push_heap`·`pop_heap`。
    即：排序、堆、二分、最短路松弛、贪心选择、图遍历的判定逻辑必须自己写。
  - ✅ **其余一律不受限**：`std::vector` / `std::map` / `std::string` 等容器、`std::min` /
    `std::max` / `std::swap` / 迭代器，以及**所有辅助功能**（io / tests / gui / 日志 / 格式化）都可随意用。
  - 完整边界与修订经过见 [`docs/设计.md`](docs/设计.md) §10.1。**不要把这条读成"核心层禁用 STL"**——那是旧口径。
- **核心层（`src/core/`、`src/io/`）不得 `#include` 任何 Qt 头文件**，必须能用 `g++` 独立编译。
- C++17、面向对象；不引入第三方 JSON/INI 库（INI 解析器是手写的）。
- `要求源文件/` 是老师给的原始文件，**只读**。

## 2. 常用命令

```bash
bash 启动软件.sh                  # 启动 GUI（首次会自动 CMake 配置 + 编译）
bash 人工测试向导.sh              # 8 关人工测试向导（需要真实终端）
(cd build && ctest)              # 全部自动化测试：20 个目标 / 827 项断言（以 ctest -N 为准）
./build/app --help               # 主程序全部命令行选项
```

## 3. 不变量（改坏了必须立刻发现）

- **三策略黄金值**（`./build/app --plan-summary distance|cost|time`）：
  - distance `178.200km / 378.300min / 244.200元 / penalty 166 / 6 趟`
  - cost `186.300km / 461.000min / 235.900元 / penalty 207 / 6 趟`
  - time `183.200km / 372.100min / 255.200元 / penalty 201 / 6 趟`
- `ctest` 全绿；9 个测试二进制自报断言合计 **827 项**（与 ctest 目标数是两个口径）。
  目标数以 `ctest -N` 为准（当前 20 个；**不要把这个数字背成硬编码**——加了测试就要同步更新文档）。
- **任何行为改动后**：跑 `ctest` + 对比上面三行黄金值。**数字变了就必须解释为什么**，
  并同步更新断言、`docs/设计.md` 的相关小节与 [`文档导航.md`](文档导航.md) 的事实。不通过不许声称完成。

## 4. 改动纪律（本项目反复踩过的坑）

1. **权威顺序**：`docs/设计.md`（设计意图）> `docs/功能实现清单.md`（要求↔实现核验）> 其他。
   注意 `设计.md` §16 是**按轮次累积的历史日志**：那章的 `✅` 表示"该轮结束时成立"，**不等于当前状态**。
2. **发现的缺陷要补自动化守卫**，并做**反向验证**（把修复回退，守卫必须失败）；否则回归会悄悄回来。
   已有先例：`tools/check_wizard_*.py`、`tools/check_wizard_*.sh`、`tools/check_doc_links.py`。
3. **守卫必须读"用户能看到的东西"**（界面文本、落盘记录、命令输出），
   不要自己把公式重算一遍去断言——本项目因此漏过真实 bug。
4. **文档里的数字必须标口径**：写清属于「已提交基线」还是「工作区未提交修订」。
   禁止"应该实现了"这类无据表述；引用行号会漂，优先用节号 + 符号名。
5. **提交与推送**：中文说明 + `fix:` / `feat:` / `docs:` / `refactor:` / `test:` 前缀。
   **一次提交只含本次修复**（不要顺手把无关改动混进去）；改动未验证通过前不要提交。
   **提交与 `git push origin main` 都不需要逐次征求用户确认**（用户 2026-09-26 明确授权：
   你认为合适即可自行提交并推送）——但要在回复里说清提交/推送了什么、为什么是这几个文件；
   跨批次的改动请用行级暂存（`git add -p`）拆开。远端 = `git@github.com:Blorg51688/logistics-route-planner.git`。
6. `docs/人工测试记录.md` 需**真人在终端**跑向导才会刷新——它现在是陈旧记录（出自修复前的向导），
   **不要把它当作有效测试证据**，也不要为了"好看"手改它。

## 5. 已知的坑（省你半小时）

- **Qt6 缺失时 CMake 静默跳过 GUI 目标**（只打印一行 STATUS），`build/app` 根本不会生成。
  任何"启动/检查"脚本都要显式检测产物并给出可操作提示。
- **工作区被移动/改名**会让 `CMakeCache.txt` 里的旧绝对路径失效，`cmake --build` 直接报错；
  `scripts/ensure_build.sh` 已处理（比对 `CMAKE_HOME_DIRECTORY` 后重建）。
- **包装脚本里构建输出一律进 stderr**：`cmake --build` 会把 `Built target ...` 写进 stdout，
  不重定向就会污染被启动程序的 stdout。
- `--help` / `--plan-summary` / `--dump-graph` / `--ui-probe` / `--render*` **不需要图形显示**；
  只有"打开交互窗口"才需要 `DISPLAY` / `WAYLAND_DISPLAY`。

## 6. 深背景按需拉取（不要往本文件里堆）

需要项目历史与踩坑细节时，**主动去取**（不自动注入，所以得有人调）：

- `omd_memory_get({ projectPath, key })`，可用键：
  `entry-points`（根目录入口结构）、`build-gotchas`（构建坑）、`test-targets`（目标数沿革与"以实测为准"口径）、
  `sim-event-model`（模拟事件模型 D24：软件内时间事件刻 + 权重 + 结算纪律）、
  `transit-buffer-model`（缓冲库存 D23 修订：满载出仓/顺路寄存/紧急单就地满足/两版取优）、
  `commit-policy`（提交/推送授权与边界）。
- `.omd/notepad.md` 的 **priority 区**（脚本安全教训：路径归属判定、`rm -rf` 穿透符号链接；
  守卫设计教训：必须证明"真的测到了东西"、且绝不能挂住而要失败）。
- `docs/设计.md` §16（P0–P35 开发史，含每个真实缺陷的根因与修法）、`.omd/handoffs/`（历次阶段交接）。
