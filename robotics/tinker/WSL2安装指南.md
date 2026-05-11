# WSL2 安装指南

> 参考来源：[Microsoft 官方文档 - 安装 WSL](https://learn.microsoft.com/en-us/windows/wsl/install)

WSL（Windows Subsystem for Linux，适用于 Linux 的 Windows 子系统）让开发者可以在 Windows 机器上同时使用 Windows 和 Linux 的强大功能。无需传统虚拟机或双系统的额外开销，即可直接在 Windows 上运行 Linux 发行版（如 Ubuntu、Debian、Kali、Arch Linux 等）及其应用、工具和 Bash 命令行。

---

## 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10 版本 2004 及更高（内部版本 19041+）或 Windows 11 |
| 权限 | 管理员权限 |

> 如果系统版本低于上述要求，请参考 [手动安装步骤](https://learn.microsoft.com/en-us/windows/wsl/install-manual)。

---

## 一、快速安装（推荐）

### 1. 以管理员身份打开 PowerShell

右键点击「开始菜单」，选择 **「Windows PowerShell（管理员）」** 或 **「终端（管理员）」**。

### 2. 执行安装命令

```powershell
wsl --install
```

该命令将自动完成：
- 启用 WSL 所需的系统功能
- 启用虚拟机平台组件
- 下载并安装 Linux 内核
- 将 WSL 2 设为默认版本
- 默认安装 **Ubuntu** 发行版

### 3. 重启计算机

安装完成后，按照提示重启计算机使配置生效。

### 4. 完成 Linux 用户初始化

重启后，Ubuntu 窗口会自动弹出，首次启动需要等待文件解压缩完成（仅首次需要，后续启动不到一秒）。随后按照提示：

1. 输入新的 **UNIX 用户名**（可与 Windows 用户名不同）
2. 设置并确认 **密码**（输入密码时终端不显示字符，属于正常现象）

---

## 二、安装常见问题处理

| 现象 | 解决方案 |
|------|----------|
| 运行 `wsl --install` 后显示帮助文本 | WSL 已安装，使用 `wsl --list --online` 查看可用发行版，再用 `wsl --install -d <发行版名称>` 安装 |
| 安装进度卡在 0.0% | 运行 `wsl --install --web-download -d <发行版名称>`，先下载再安装 |
| 其他安装问题 | 参考 [WSL 疑难解答](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting#installation-issues) |

---

## 三、安装指定 Linux 发行版

### 查看可用发行版列表

```powershell
wsl --list --online
```

### 安装指定发行版

```powershell
wsl --install -d <发行版名称>
```

常用发行版示例：

```powershell
wsl --install -d Ubuntu          # Ubuntu（默认）
wsl --install -d Ubuntu-22.04   # Ubuntu 22.04 LTS
wsl --install -d Debian          # Debian
wsl --install -d kali-linux      # Kali Linux
```

---

## 四、管理 WSL 版本

### 查看已安装的发行版及 WSL 版本

```powershell
wsl --list --verbose
# 或简写
wsl -l -v
```

输出示例：

```
  NAME      STATE           VERSION
* Ubuntu    Running         2
  Debian    Stopped         1
```

### 将指定发行版从 WSL 1 升级到 WSL 2

```powershell
wsl --set-version <发行版名称> 2
# 例如：
wsl --set-version Ubuntu 2
```

### 设置新安装发行版的默认 WSL 版本

```powershell
wsl --set-default-version 2
```

### 设置默认使用的发行版

```powershell
wsl --set-default <发行版名称>
# 例如：
wsl --set-default Ubuntu
```

---

## 五、启动与使用 WSL

以下几种方式均可启动 WSL：

| 方式 | 说明 |
|------|------|
| 开始菜单搜索 | 搜索发行版名称（如"Ubuntu"），点击即可在独立窗口打开 |
| PowerShell 中输入发行版名称 | 如直接输入 `ubuntu` |
| `wsl.exe` | 在 PowerShell 中打开默认发行版 |
| `wsl [命令]` | 在当前命令行中直接执行 Linux 命令，如 `wsl ls -la` |
| **Windows Terminal（推荐）** | 支持多标签、多窗格，可快速切换不同发行版 |

退出 WSL 命令行：

```bash
exit
```

---

## 六、运行多个 Linux 发行版

WSL 支持同时安装并运行多个 Linux 发行版，互不干扰。可通过以下途径获取发行版：

- **Microsoft Store**：搜索对应发行版名称直接安装
- **命令行**：使用 `wsl --install -d <名称>`
- **自定义导入**：通过 TAR 文件导入任意 Linux 发行版

```powershell
# 导入自定义发行版示例
wsl --import <发行版名称> <安装路径> <tar文件路径>
```

---

## 七、验证安装

安装完成后，可通过以下命令验证 WSL 2 是否正常运行：

```powershell
# 查看 WSL 版本信息
wsl --version

# 查看所有已安装发行版
wsl -l -v

# 在 WSL 中执行命令测试
wsl uname -a
```

---

## 八、（可选）体验 WSL 预览功能

如需体验最新 WSL 预览特性，可执行：

```powershell
wsl --update --pre-release
```

或加入 [Windows 预览体验计划](https://www.microsoft.com/windowsinsider/getting-started)，选择合适的通道：

| 通道 | 适合人群 | 稳定性 |
|------|----------|--------|
| Canary | 高级技术用户 | 最低，无完整文档 |
| Dev | 发烧友 | 较低，功能最新 |
| Beta | 早期采用者 | 较稳定 |
| Release Preview | 商业用户、普通用户 | 接近正式版 |

---

## 附：常用 WSL 命令速查

```powershell
wsl --install                        # 安装 WSL（默认 Ubuntu）
wsl --install -d <名称>              # 安装指定发行版
wsl --list --online                  # 查看可用发行版
wsl -l -v                            # 查看已安装发行版及版本
wsl --set-default-version 2          # 设置默认 WSL 版本为 2
wsl --set-version <名称> 2           # 将指定发行版升级到 WSL 2
wsl --set-default <名称>             # 设置默认发行版
wsl --update                         # 更新 WSL
wsl --shutdown                       # 关闭所有运行中的发行版
wsl --unregister <名称>              # 卸载指定发行版
```

---

> **下一步建议**：参考 [WSL 开发环境最佳实践](https://learn.microsoft.com/en-us/windows/wsl/setup/environment) 配置 Git、VS Code 远程开发、数据库等开发工具。
