# 本地主臂 SDK 修复

基于 PyPI `motorbridge-smart-servo==0.0.4` 源码包，对应上游 v0.0.4（319649f0b89d0aa247d8862ca4c4c72c198e6d90）。原包元数据声明 MIT，作者 tianrking。上游：https://github.com/motorbridge/motorbridge-smart-servo 。本地版本号 `0.0.4+rsfix1`，未发布到 PyPI。

`rsfix1.patch` 提供完整改动（含为此源码包重新生成的 Cargo.lock），源码归档可直接构建。原 SDK 一旦接受零度，后续仍每 30 帧重新确认；此修复使已接受的近零角持续可靠。非零跳到零仍需确认；新增 `set_zero_confirm_samples()` 调整现有及未来关节过滤器并返回旧设置。本适配只在会话内设置为 3，退出恢复。其他使用 SDK 的程序仍使用默认 30 帧确认。

没有删除校验和、原始有效范围、缺包缓存标记或丢包计数器。不能将此修改理解为所有零值都可放行。

提供的 wheel 适用于 Linux x86_64、glibc >=2.34、Python >=3.9；已在用户 Python 3.12 环境加载。其他平台从源码构建：

```bash
python -m pip install 'maturin>=1.7,<2'
# 解压源码，进入包含 pyproject.toml 的目录（需 Rust 工具链和 C 链接器）
cargo test -p smart_servo_core -p smart_servo_vendor_fashionstar
maturin build --release --interpreter python --out dist
```

需要回到上游原版时，在停止遥操作后执行 `python -m pip install --no-deps --force-reinstall motorbridge-smart-servo==0.0.4`；回退后本适配 0.1.2 会提示缺少修复 API，不能继续用本包。

验证：15 项 Rust 测试通过；Python PTY 测试覆盖持续零度、单关节缺包、校验和损坏、3 帧零跳变确认。测试只访问伪终端，不连接实际串口。
