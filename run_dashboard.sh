#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# 仅启动 localhost 网页；在页面点“启动反馈”后才连接机械臂。
python -m stararm_rs_feedback.dashboard --root "$PWD" --port "${1:-8765}"
