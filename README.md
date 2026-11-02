# qiyu-head-tracking

QVR/CDSP 头部跟踪与姿态接口。

实际功能源码首次按仓库所有权整理，来源和导入前后 SHA-256 见 `provenance/source-import.json`。原有历史构建与设备结果不自动等同于拆分后的设备验收。

构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [qiyu-docs](https://github.com/bitsk/qiyu-docs)。

## 构建

相邻检出所需组件，并指定已准备的 qiyu-kernel 检出：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
  -DQIYU_KERNEL_DIR=/absolute/path/to/qiyu-kernel
cmake --build build -j4
```

头部插件把统一采集作为库链接到唯一生产者，控制器和调试端消费其接口；这次不新增竞争采集进程。第三方 HexagonRPC 代码保留 COPYING，SDK/DSP 固件在设备运行时另行导入。
