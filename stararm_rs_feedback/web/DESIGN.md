# 界面实现

Star Arm 102 工作台采用白色界面、黑色主操作和蓝色交互反馈。曲线颜色区分关节，模型受力颜色表示扭矩幅值，二者使用不同的视觉编码。

## 交互

工作台、调校和日志采用标签视图。完整视图切换通过 Web Animations 处理位置、尺寸和透明度变化，保留真实 DOM 以维持操作响应；局部内容替换支持 View Transition，并提供兼容过渡。

按钮、菜单、诊断区、日志、提示、加载和标注使用统一的进入 / 退出生命周期。连续操作会取消旧动画并保留最后状态。动效通常为 160–420 ms，支持 `prefers-reduced-motion`。

实时扭矩数值直接显示最新反馈，不做数值插值。数据失效时立即清空图上读数；模型姿态、材质、标签位置和相机复位仅作视觉平滑，不改变设备命令。

标签支持方向键及 Home / End，量程菜单支持键盘选择；曲线支持指针和左右方向键查看采样，Escape 关闭读数。退出中的控件处于 inert 状态。

## 资源

- Fashionstar 品牌图标：见 [brand/NOTICE.txt](brand/NOTICE.txt)
- Lucide SVG 图标：取自 [Lucide](https://github.com/lucide-icons/lucide/tree/main/icons)，ISC / MIT 授权见 [vendor/LICENSE-lucide.txt](vendor/LICENSE-lucide.txt)
- Three.js 和模型来源：见仓库 [NOTICE.md](../../NOTICE.md)

网页资源随安装包提供，不依赖远程字体、图标 CDN 或在线组件服务。

## 参考

交互设计参考 [beUI](https://beui.dev/)、[Transitions.dev](https://transitions.dev/)、[shadcn/ui](https://ui.shadcn.com/) 和 [Rare UI](https://rareui.com/)。动效逻辑位于 `motion.js`，未引入上述项目的付费组件。
