# 第三方来源

- RS URDF 与展示 STL 来源于 Seeed Studio 的 reBot Arm 资料，保留 `stararm_rs_feedback/assets/LICENSE-Seeed.txt` 与 `stararm_rs_feedback/web/model/LICENSE-Seeed.txt`。
- Three.js、OrbitControls、STLLoader 的 MIT 授权保留于 `stararm_rs_feedback/web/vendor/LICENSE-three.txt`。
- Lucide SVG 图标来源于 Lucide 官方仓库，ISC / MIT 授权保留于 `stararm_rs_feedback/web/vendor/LICENSE-lucide.txt`。
- Fashionstar 品牌标识用于 Star Arm 102 产品界面，品牌权利归相应权利人所有，不属于第三方图标库授权范围。
- `motorbridge-smart-servo` 修复版基于 PyPI 0.0.4，上游声明 MIT，作者 tianrking。上游链接、修复差异、源码构建说明见 `sdk-fix/README.md`。随包提供源码归档与本地构建的 wheel。
- 本项目与 VincentHSS/stararm102-force-feedback 的目标一致；本实现针对外置 LeRobot RS 插件，不使用原项目的 DM 通信控制路径。

这些第三方授权仅覆盖各自文件；本仓库没有为其他代码额外声明开源许可证。
