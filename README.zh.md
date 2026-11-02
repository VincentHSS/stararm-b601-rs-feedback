# Star Arm 102 力反馈工作台

<sub>[English](README.md) / **简体中文**</sub>

面向 **Star Arm 102 Leader + Seeed reBot B601 RobStride 从臂**的本地遥操作与力反馈工具。支持动态阻尼、堵转保持、三维姿态显示和网页调参。

当前源码版本 **0.6.2** · 请从当前源码安装；现有 Release 不包含本次改动 · [更新记录](CHANGELOG.md)

## 主要功能

- **动态力反馈**：根据从臂当前负载调整主臂阻尼，支持独立堵转 HOLD
- **实时工作台**：三维模型、七轴状态和扭矩趋势同屏显示
- **图上受力**：关节扭矩数值与连杆着色，支持电机扭矩、模型剩余力矩两种视图
- **七轴曲线**：各轴独立配色，支持关节联动、采样读数和历史日志回放
- **在线调参**：设置共同及各关节阻尼功率上限；死区、运动余量、力矩标尺和响应参数固定显示
- **中英文界面**：顶部“中文 / EN”切换，当前浏览器记住语言；切换不改变参数
- **本地运行**：网页、模型、图标和 Three.js 均随包提供，无需外部 CDN

J1–J6 和夹爪参与普通动态阻尼，J6 按自身腕部回转剩余力矩反馈。普通阻尼总上限默认 1200 mW，可设为 400–1500 mW；HOLD 独立，可对全部关节请求 1500 mW。旧网页将 J6 上限保存为 0 时保留关闭状态，可调高后保存。

## 环境要求

- Linux、Python 3.12、LeRobot 及双方设备插件
- 已完成串口、SocketCAN、方向映射和校准，基础遥操作可正常跟随
- 桌面 Chrome / Chromium 或其他支持 WebGL 的浏览器

设备配置参考 [Seeed B601-RS 教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/)。本项目使用外置设备类型 `rebot_arm_102_leader` 和 `seeed_b601_rs_follower`，不适用于 B601-DM。

## 安装

```bash
git clone https://github.com/VincentHSS/stararm-b601-rs-feedback.git
cd stararm-b601-rs-feedback
conda activate lerobot
python -m pip install --no-deps dist/motorbridge_smart_servo-0.0.4+rsfix1-*.whl
python -m pip install --no-deps .
```

[Releases](https://github.com/VincentHSS/stararm-b601-rs-feedback/releases) 与 `dist/` 中的 wheel 为旧版本。本次只更新源码，使用上方源码安装命令获取这些改动。

附带的主臂 SDK 修复包适用于 Linux x86_64、glibc ≥ 2.34、Python ≥ 3.9。其他平台和构建说明见 [sdk-fix/README.md](sdk-fix/README.md)。环境缺少 python-can 时，安装 `python-can>=4.2,<5`。

## 首次绑定

固定底座保持正立，末端使用原装夹爪。按实际校准文件路径生成本机配置：

```bash
python -m stararm_rs_feedback.bind \
  --calibration /实际路径/follower1.json \
  --robot-id follower1 \
  --upright-stock-gripper \
  --output feedback.rs-dynamic.json
```

绑定过程只读取配置与既有校准，不连接设备、不重新置零，也不覆盖已有文件。校准或映射变更后需重新绑定。`feedback.rs-dynamic.example.json` 仅用于说明结构，不能直接用于硬件。

升级已有环境时，保留本机 `feedback.rs-dynamic.json` 和 `data/`。从 0.5.1 及更早版本升级时，在绑定配置的 `policy` 中补充 `"arm_motion_range_reset": false`，保留原有签名、校准和其他字段。

从 0.6.1 或更早版本升级到本次 J6 策略时，先备份并移开旧绑定，再用相同的既有校准执行上方“首次绑定”。绑定在离线完成，不重新校准、不设置零位。不要手改哈希绕过策略检查。

## 启动工作台

```bash
conda activate lerobot
bash run_dashboard.sh
```

浏览器打开 <http://127.0.0.1:8765/>。点击顶部“中文 / EN”切换界面语言。需要更换端口时运行 `bash run_dashboard.sh 8766`。

启动网页服务不会连接设备。退出其他遥操作会话后，点击“启动反馈”；“仅监测”仍会启动从臂跟随，但不向主臂发送阻尼或 HOLD。默认设备为 `can0`、`/dev/ttyUSB0`，从臂 ID 为 `follower1`，主臂 ID 为 `rebot_arm_102_leader`，控制频率为 30 Hz。

CAN 尚未启用时，停止遥操作后按实际适配器配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

网页只允许调整共同及各轴功率上限；调大允许更强阻尼，调小限制强度。死区、运动余量、满输出标尺和响应参数固定显示。降低共同上限会同步降低超出的每轴上限；提高共同上限不会自动提高每轴上限。“恢复默认上限”只恢复功率上限，保留固定参数。

网页参数保存在本机 `data/web/`。点击“保存并应用”后，以“已应用 vN”确认控制循环已接收参数。详细操作见 [WEB.md](WEB.md)。

## 终端运行

```bash
bash run_force_feedback.sh       # 默认上限 1200 mW
bash run_force_feedback.sh 1500  # 上限 1500 mW，两条命令任选其一
```

常用总阻尼、各关节默认上限及七轴阈值位于 `stararm_rs_feedback/tuning.py` 开头；模型与通信策略仍位于 `dynamic.py` 的 `POLICY`。直接修改策略时，需同步本机绑定配置中的 `policy`；网页调参单独持久化，无需修改源码或绑定配置。详见 [DYNAMIC.md](DYNAMIC.md)。

## 使用说明

- 扭矩单位为 N·m，属于电机反馈估计；Iq 为电机滤波电流。两者都不能直接换算为末端接触力或公斤数
- 网页中的 mW 是 Leader 请求功率，不是实测阻力。曲线颜色用于区分关节，模型颜色表示显示量程内的扭矩幅值，不是安全限值
- “停止运行”发送 SIGINT，执行正常回零或卸力清理。操作前留出空间并支撑机械臂；关闭网页标签不会停止会话
- 更新安装包不会替换正在运行的进程。升级后正常停止旧会话，再重启服务并刷新页面
- CAN 时效、故障、温度、位置跳变、限位、Leader 连续缺帧及近零姿态检查保持生效。当前策略在超过旧臂部运动模型范围时继续计算阻尼，模型误差可能增大

## 测试与资料

```bash
python -B -m unittest discover -s tests -q
node tests/test_torque.mjs
node tests/test_motion.mjs
node tests/test_presentation.mjs
node tests/test_i18n.mjs
```

Python 测试使用设备替身与伪终端，完整拟合测试需要 NumPy，原生传输测试需要修复版 SDK。当前版本的验证范围、历史回放记录和真机验证边界见 [VALIDATION.md](VALIDATION.md)。

- [网页操作](WEB.md)
- [动态策略](DYNAMIC.md)
- [更新记录](CHANGELOG.md)
- [界面实现](stararm_rs_feedback/web/DESIGN.md)
- [第三方来源与授权](NOTICE.md)
- [Star Arm 102 原力反馈项目](https://github.com/VincentHSS/stararm102-force-feedback)
