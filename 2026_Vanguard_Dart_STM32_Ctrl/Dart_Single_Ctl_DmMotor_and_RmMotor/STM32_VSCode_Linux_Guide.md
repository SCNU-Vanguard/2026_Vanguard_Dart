# Linux 下 STM32 VS Code 开发指南

本文档总结了在 Linux 环境下使用 **STM32 VS Code Extension** 进行开发的注意事项、通用配置模板以及 CMSIS-DAP 调试器的详细设置方法。

## 1. STM32 VS Code Extension 使用避坑指南

该插件核心是基于 **Cortex-Debug** 和 **CMake** 的封装。在 Linux 下使用时，建议遵循以下最佳实践以避免常见问题。

### A. 环境变量与路径
*   **问题**：插件默认倾向于使用 ST 官方的 `CubeCLT` 路径，可能无法识别 Linux 系统安装的工具。
*   **建议**：
    *   使用系统包管理器安装核心工具：`sudo apt install openocd gdb-multiarch`。
    *   在配置中**硬编码**系统路径（如 `/usr/bin/openocd`），比依赖插件的自动检测更稳定。

### B. 调试器选择 (OpenOCD vs ST-LINK)
*   **OpenOCD (推荐)**：
    *   支持多种调试器 (CMSIS-DAP, J-Link, ST-Link)。
    *   对 **Live Watch (实时变量监控)** 支持极佳。
*   **ST-LINK GDB Server**：
    *   ST 官方工具，对 ST-Link 固件支持好，但 Live Watch 功能往往不可用或不稳定。

### C. SVD 文件 (寄存器视图)
*   **作用**：让调试界面的 "Peripherals" 窗口显示芯片外设寄存器的具体值。
*   **操作**：建议下载对应芯片的 `.svd` 文件放入项目目录（如 `.vscode/` 下），并在 `launch.json` 中配置引用，确保项目移植性。

---

## 2. Linux 通用 `launch.json` 模板

将以下内容覆盖或合并到你的 `.vscode/launch.json`。此模板优化了路径配置，移除了对特定插件变量的过度依赖。

**使用前请修改标注 `TODO` 的部分** (如 `device`, `configFiles`, `svdFile`)。

```json
{
    "version": "0.2.0",
    "configurations": [
        /* ------------------------------------------------------------------
         * 配置 1: OpenOCD (推荐 - 通用性强，支持 Live Watch)
         * 适用于: CMSIS-DAP, ST-Link, J-Link
         * ------------------------------------------------------------------ */
        {
            "name": "Linux - OpenOCD (Universal)",
            "cwd": "${workspaceFolder}",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            
            // 自动获取 CMake 生成的 ELF 文件路径
            "executable": "${command:cmake.launchTargetPath}",
            
            // TODO: 修改为你的具体芯片型号
            "device": "STM32F427IIHx", 
            
            // Linux 系统 OpenOCD 默认路径
            "serverpath": "/usr/bin/openocd",
            
            // TODO: 根据调试器和芯片修改
            // 接口: interface/cmsis-dap.cfg (DAP-Link), interface/stlink.cfg (ST-Link)
            // 目标: target/stm32f4x.cfg (F4系列), target/stm32f1x.cfg (F1系列)
            "configFiles": [
                "interface/cmsis-dap.cfg", 
                "target/stm32f4x.cfg"
            ],

            // 开启实时变量监控 (无需暂停核心即可看变量变化)
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            },

            // TODO: 指定 SVD 文件路径
            "svdFile": "${workspaceFolder}/.vscode/STM32F427.svd", 
            
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none",
            "gdbPath": "gdb-multiarch" 
        },

        /* ------------------------------------------------------------------
         * 配置 2: ST-Link GDB Server (官方原生)
         * 适用于: 仅限 ST-Link 调试器
         * ------------------------------------------------------------------ */
        {
            "name": "Linux - ST-Link Official",
            "cwd": "${workspaceFolder}",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "stlink",
            "executable": "${command:cmake.launchTargetPath}",
            "device": "STM32F427IIHx",
            "interface": "swd",
            "serverpath": "ST-LINK_gdbserver", // 需确保在 PATH 中或写绝对路径
            "stm32cubeprogrammer": "STM32_Programmer_CLI",
            "runToEntryPoint": "main",
            "svdFile": "${workspaceFolder}/.vscode/STM32F427.svd",
            "gdbPath": "gdb-multiarch",
            "serverArgs": ["-m", "0"]
        }
    ]
}
```

---

## 3. CMSIS-DAP Link 配置指南

CMSIS-DAP 是一种通用的开源调试协议，无需专有驱动，配合 OpenOCD 使用效果极佳。

### 3.1 硬件连接 (SWD)

| CMSIS-DAP 引脚 | STM32 引脚 | 备注 |
| :--- | :--- | :--- |
| **SWCLK** | **SWCLK (PA14)** | 时钟 |
| **SWDIO** | **SWDIO (PA13)** | 数据 |
| **GND** | **GND** | **必须共地** |
| **3V3** | **3V3** | 仅当需要调试器供电时连接 |

### 3.2 Linux 权限配置 (udev rules)

为了让非 root 用户（VS Code）能访问 USB 调试器，必须配置 udev 规则。

1.  **创建规则文件**:
    ```bash
    sudo nano /etc/udev/rules.d/99-cmsis-dap.rules
    ```

2.  **写入以下内容**:
    ```bash
    # CMSIS-DAP compatible adapters
    SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE="0666"
    SUBSYSTEM=="usb", KERNEL=="hidraw*", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", MODE="0666"
    SUBSYSTEM=="usb", ATTRS{product}=="*CMSIS-DAP*", MODE="0666"
    ```

3.  **重载并生效**:
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    *提示：执行后请重新插拔调试器。*

### 3.3 验证连接

在终端运行以下 OpenOCD 命令测试连接（以 STM32F4 为例）：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg
```

*   **成功**: 输出包含 `Info : STM32F4xx flash size is ...`。
*   **失败**: 提示 `Error: open failed` (检查权限) 或 `target not found` (检查接线)。
