# 定量的探针（一次性分析件，未入库）

来源：`../transit-stock-full-load.md` 四轮商议中的实测装置。**只读，不参与构建。**

编译（在仓库根目录执行；核心层无 Qt，可独立编译）：

```bash
R="$PWD"
g++ -std=c++17 -O1 -I "$R/src" docs/留档/决策/probes/probe.cpp  "$R"/src/core/*.cpp "$R"/src/io/*.cpp -o /tmp/probe_abc
g++ -std=c++17 -O1 -I "$R/src" docs/留档/决策/probes/probeD.cpp "$R"/src/core/*.cpp "$R"/src/io/*.cpp -o /tmp/probe_d
g++ -std=c++17 -O1 -I "$R/src" docs/留档/决策/probes/probeE.cpp "$R"/src/core/*.cpp "$R"/src/io/*.cpp -o /tmp/probe_e
g++ -std=c++17 -O1 -I "$R/src" docs/留档/决策/probes/probeF.cpp "$R"/src/core/*.cpp "$R"/src/io/*.cpp -o /tmp/probe_f
```

运行（均以 `config/default.ini` 为输入）：

```bash
/tmp/probe_abc config/default.ini   # A: 每趟返程盈余 / B: 回程是否顺路 / C: 紧急单现状代价与经站上界
/tmp/probe_d   config/default.ini   # D1: 从站取货送客户的绕路 / D2: 可寄存量
/tmp/probe_e   config/default.ini   # E: 车上现成货满足紧急单的乐观上界
/tmp/probe_f   config/default.ini   # F1: 两种货源读法下能攒多少 / F2: 用户例子的收益
```

**这些探针是"改动前"的测量**：一旦 `src/` 被实现对话修改，A/B/C/D/E/F 的数字会随之变化。
若要复现商议记录里的数字，请对 `HEAD = db08918` 的源码树运行（例如先用 `git archive` 取一份到 /tmp）。
