# OmniBotHmi — 地面站/上位机

**路径**: `OmniBotHmi/`

---

## OmniRobHmi.exe (OCU 地面站)

- **路径**: `OCU_software/OmniRobHmi.exe`
- **平台**: Windows x64, PyInstaller 打包
- **大小**: 9.4MB
- **运行时**: Python 3.13 + SDL2 + NumPy + Tkinter

### 功能

- UDP 连接机器人 (Odroid C4) 进行遥控和状态监控
- 支持多机器人 (ocu_param.txt 配置最多 4 个 IP)
- SDL2 图形渲染 + Tkinter 界面

### 配置文件 (`OmniBotHmi/ocu_param.txt`)

```
UDP_IP_Robo1, 192.168.1.188
UDP_IP_Robo2, 192.168.1.244
UDP_IP_Robo3, 192.168.1.113
UDP_IP_Robo4, 192.168.1.128
```

---

## 调试面板 (py_ui.py)

- **路径**: `OmniBotCtrl/tools/py_ui.py`
- **平台**: Linux, Tkinter GUI
- **通信**: UDP port 8001 → 192.168.1.246
- **功能**: 使能/RC/Idle/Walk/Start/Stop/Disable 按钮, WASD 速度控制, J/L 转向
- 来源: "Nabo (小炮)" 开源双足项目 (Shanghai Jiao Tong University, MIT License)

## Xbox 手柄遥控 (py_xbox.py)

- **路径**: `OmniBotCtrl/tools/py_xbox.py`
- **平台**: Linux, pygame
- **通信**: 同 py_ui.py UDP 协议
- **功能**: 左摇杆=速度, 扳机=转向, ABXY/DPad=模式切换
