# StarArm 102 + B601-RS 动态力反馈与本地网页

适配 **StarArm 102 Leader + Seeed reBot B601 RobStride 从臂**，复用 Seeed LeRobot 官方设备插件。0.5.4 提供随当前负载变化的主臂阻尼、本地网页调参、RS 3D 姿态、图上七轴扭矩标注与变色、Iq 电流及 Leader 请求状态。

J1–J5 和夹爪参与普通动态阻尼，J6 普通阻尼关闭；默认每轴上限 1200 mW，可选 400–1500 mW。J1 只按自身回转负载判断。堵转 HOLD 独立，可对全部关节请求 1500 mW。这里的 mW 是 Leader 请求功率，不是实测阻力；扭矩/电流不能直接换算公斤。

## 0.5.4 精简数据与同屏曲线

移除模型下方重复角度和关节表里的重复扭矩，曲线移到右侧关节表下方。桌面可同屏观察模型、七轴状态和扭矩曲线；选中关节的温度、剩余力矩、死区与 Leader 角度收进默认折叠的“诊断详情”。

## 0.5.3 图上受力与新版工作台

浅色工作台放大 3D 模型，J1–J7 直接显示有符号扭矩（N·m），用绿→黄→红表示幅值，并对对应连杆着色；标注跟随姿态和视角，点击可联动关节详情。支持切换电机扭矩 / 模型剩余力矩和调整颜色量程，失效或过期数据变灰并显示 `—`，回放明确标为历史数据。颜色量程只影响显示，不是额定/安全限值；反馈扭矩也不是末端接触力。详见 [网页说明](WEB.md)。

本次仅更新网页及显示版本，机械臂控制策略不变。请从源码安装 0.5.4；`dist/` 中历史 wheel 不含此次界面更新。

## 0.5.2 持续运动时保持阻尼计算

临时关闭 `POLICY['arm_motion_range_reset']`：超过旧的臂部速度/加速度模型阈值时，继续按当前 RS 状态计算残差和平滑功率，不再全臂清零并重新初始化。夹爪策略、各轴普通功率上限和独立 HOLD 不变。CAN 时效、连续 Leader 缺帧超时、故障、温度、位置跳变、限位和近零位模型检查继续生效。

同一段约 65 秒、1947 帧日志离线回放中，运动超范围引起的骤降由 5 次降至 0 次，总骤降由 8 次降至 3 次；剩余来自限位/近零位。148 项离线测试通过。用户已反馈本地试用可用，仍未进行长期稳定性或外力计量验证；超过原运动范围时模型误差可能增大。

本次只更新源码、测试及文本说明，`dist/` 中历史 wheel 保持原样。请按下方命令从源码安装 0.5.2。升级已有本机绑定配置时，仅在其 `policy` 中补上 `"arm_motion_range_reset": false`，保留原有签名、校准和其他字段；全新使用者按“首次离线绑定”生成配置即可。恢复旧运动重置行为时，将 `dynamic.py` 和本机绑定配置中的这个字段同时改为 `True` / `true`。

## 0.5.1 修复带载时反复松开

旧版把某个 Leader 轴短暂不可靠读数当成全轴动态模型失效，所有阻尼清零后重新初始化。新版在此期间暂停向从臂发新目标，同时继续用新鲜 RS 状态更新阻尼；卸载时功率继续下降，不锁住旧负载。缺帧不累计堵转或解除已有 HOLD。

持续 Leader 缺帧 0.25 秒仍退出释放。CAN 时效、故障、温度和各轴限位检查保留。距本轴软件限位小于 2° 时释放该轴，离开到 3° 后恢复；J2/J3 电机角均小于 3° 时暂停 J2–J5 模型反馈，任一达到 5° 后恢复。保护性释放仍可能发生。

软件测试与离线日志回放通过，用户已反馈本地试用可用。URDF 自重/惯性估计含摩擦与模型偏差，不能保证全姿态空载完全无阻力，也不是精确外力反射。详见 [VALIDATION.md](VALIDATION.md)。

## 安装

先按 [Seeed B601-RS 教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/) 配好 Python 3.12 / LeRobot、双方设备插件与既有校准，确认官方遥操作能正常跟随。此项目不替代设备安装、方向核对或校准。

```bash
git clone https://github.com/VincentHSS/stararm-b601-rs-feedback.git
cd stararm-b601-rs-feedback
conda activate lerobot
python -m pip install --no-deps dist/motorbridge_smart_servo-0.0.4+rsfix1-*.whl
python -m pip install --no-deps .
```

所附主臂 SDK wheel 适用于 Linux x86_64、glibc >=2.34、Python >=3.9；修复已接受零度反复被拒绝的问题，仍保留校验和、缺包及零跳变确认。其他平台见 [sdk-fix/README.md](sdk-fix/README.md)。设备插件仍由教程环境提供；如缺 python-can，单独安装 `python-can>=4.2,<5`。本包 0.5.4 从当前源码安装；`dist/` 内旧 0.5.1 wheel 没有本次改动。

使用的外置设备类型为 `rebot_arm_102_leader` 与 `seeed_b601_rs_follower`，不适用于 LeRobot 内置的同名近似插件或 B601-DM。

## 首次离线绑定

公开仓库不包含任何机器的校准、运行日志或本地参数。固定底座必须正立、末端为原装夹爪。确认官方既有校准后，用实际 follower 校准文件路径生成本机配置：

```bash
python -m stararm_rs_feedback.bind \
  --calibration /实际路径/follower1.json \
  --robot-id follower1 \
  --upright-stock-gripper \
  --output feedback.rs-dynamic.json
```

此命令只读取配置与既有校准，不连接设备，不重新置零，不覆盖已有文件；绑定检查身份和映射，不证明校准质量。启动脚本使用官方默认 RS 映射、限位和型号；自行修改设备映射的用户应同步配置并重新核对，不能复制其他机器的签名。校准/映射改变后旧配置会拒绝启动。

`feedback.rs-dynamic.example.json` 只展示结构，不能直接用于硬件。已拥有本机绑定配置的升级用户继续保留自己的 `feedback.rs-dynamic.json` 和 `data/`。

## 启动网页

```bash
conda activate lerobot
cd /path/to/stararm-b601-rs-feedback
bash run_dashboard.sh
```

打开 <http://127.0.0.1:8765/>。网页服务本身不连接硬件，点击“启动反馈”才启动遥操作；“仅监测运行”仍使从臂跟随，但不向主臂写入阻尼。先退出其他遥操作会话。默认使用 `can0`、`/dev/ttyUSB0`、从臂 ID `follower1`、主臂 ID `rebot_arm_102_leader`、30 Hz；设备名应与既有校准环境一致。

CAN 尚未启动时，停止遥操作后按实际适配器配置，例如：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

网页支持每轴上限、死区、运动余量、满输出标尺、滤波与响应速度。点击“保存并应用”后看“已应用 vN”；改参会重建普通估计器并短暂释放/重新确认。参数保存在本机 `data/web/`，所有网页/Three.js/STL 资源均本地加载。详细说明见 [WEB.md](WEB.md)。

安装更新不会改变已运行的反馈进程。正常停止旧反馈后再启动；重启网页服务并刷新页面可更新版本显示。停止按钮按 Ctrl+C 正常清理，官方可能回零/卸力，需要留出空间并支撑机械臂；关闭标签页不会停止会话。

## 终端与源码调参

```bash
bash run_force_feedback.sh          # 默认 1200 mW
```

上面的启动命令与 `bash run_force_feedback.sh 1500`（1500 mW 上限）任选一条。优先在网页修改参数；终端脚本的默认上限在 `run_force_feedback.sh`。每轴死区/标尺及默认策略在 `stararm_rs_feedback/dynamic.py` 的 `POLICY`，均有中文注释。直接改默认策略时必须同步绑定 spec 的 `policy`；严格匹配会拒绝不一致配置。网页参数单独持久化，不要求改源码/JSON。HOLD 上限由 `config.py` 的 `hold_mw` 决定，和普通上限独立。详见 [DYNAMIC.md](DYNAMIC.md)。

## 离线测试

```bash
python -B -m unittest discover -s tests -q
```

测试使用设备替身与伪终端，不打开真实串口或 CAN；完整拟合测试需要 NumPy。网页/故障保护测试可离线运行，原生传输测试需所附修复 SDK。源码还保留历史静止/空载模型工具供研究，当前推荐入口是上述动态模式。

参考：[原力反馈项目](https://github.com/VincentHSS/stararm102-force-feedback)、[Seeed RS 教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/)。第三方模型和库的归属见 [NOTICE.md](NOTICE.md)。
