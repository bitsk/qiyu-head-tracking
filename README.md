# qiyu-head-tracking

QVR/CDSP 头部跟踪与姿态接口。


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

## 原厂运行资源

DSP runtime 和型号默认 config6 已归档到独立公开 [qiyu-vendor](https://github.com/bitsk/qiyu-vendor) Release。通过 qiyu-rootfs 的 `tools/vendor.py fetch` 下载并校验后，缓存里的 `qvr/runtime/qcom/` 提供 DSP 文件，`qvr/config/config6.xml` 提供型号默认配置；rootfs 安装目标记录在 vendor.lock.json 中。每台设备的相机校准独立提供，不从其他设备的 persist 复制。正常使用不依赖整份原厂镜像。
