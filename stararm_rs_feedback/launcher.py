import argparse
import functools
import importlib
import inspect
import sys
from pathlib import Path

from .config import load
from .session import Session


def install_runtime_wrapper(module, cfg, *, enabled=False, log_path=None, collect_path=None, session_factory=Session):
    """只包装本次进程，退出即可恢复。两个设备在官方循环开始前已经连接。"""
    original = module.teleop_loop
    if getattr(original, "_stararm_rs_feedback", False):
        raise ValueError("RS feedback wrapper already active")
    signature = inspect.signature(original)
    if not {"teleop", "robot"} <= set(signature.parameters):
        raise ValueError("Unsupported LeRobot teleop_loop signature")
    try:
        source = inspect.getsource(original)
    except (OSError, TypeError):
        source = ""
    if "_stararm_automatic_feedback" in source:
        raise ValueError("Remove legacy stararm_auto_feedback hook before using the RS launcher")

    @functools.wraps(original)
    def wrapped(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        options = dict(enabled=enabled, log_path=log_path)
        if collect_path:
            options['collect_path'] = collect_path
        with session_factory(bound.arguments["teleop"], bound.arguments["robot"], cfg, **options):
            return original(*args, **kwargs)

    wrapped._stararm_rs_feedback = True
    module.teleop_loop = wrapped
    return original


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="StarArm 102 + B601-RS: monitor by default; official LeRobot controls follower.",
        epilog="Remaining --robot.*, --teleop.*, --fps etc. are forwarded to LeRobot.",
        allow_abbrev=False)
    parser.add_argument("--feedback-config", help="RS feedback JSON file")
    parser.add_argument("--feedback-enable", action="store_true", help="Enable physical Leader damping/hold")
    parser.add_argument("--feedback-log", help="Create a NEW CSV diagnostic file (never overwrite)")
    parser.add_argument("--collect-unloaded", help="Record a NEW raw RS CSV during monitor teleoperation")
    parser.add_argument("--confirm-unloaded", action="store_true",
                        help="Confirm entire capture is unloaded, upright, stock gripper and no external contact")
    args, remaining = parser.parse_known_args(argv)
    cfg = load(args.feedback_config)
    if args.collect_unloaded:
        if not args.confirm_unloaded or args.feedback_enable or cfg.unloaded_profile:
            parser.error("Capture requires --confirm-unloaded, monitor mode, and no unloaded profile")
        path = Path(args.collect_unloaded)
        if path.exists() or path.with_suffix('.meta.json').exists():
            parser.error("Capture output already exists; choose a new file")
    elif args.confirm_unloaded:
        parser.error("--confirm-unloaded requires --collect-unloaded")
    # 延迟导入，--help 与无硬件测试不需要安装整个 LeRobot/驱动环境。
    module = importlib.import_module("lerobot.scripts.lerobot_teleoperate")
    original = module.teleop_loop
    original_factory = None
    saved_argv = sys.argv
    try:
        install_runtime_wrapper(module, cfg, enabled=args.feedback_enable, log_path=args.feedback_log,
                                collect_path=args.collect_unloaded)
        if cfg.unloaded_profile:
            from .unloaded import Profile
            profile = Profile(cfg.unloaded_profile)
            original_factory = module.make_robot_from_config

            def checked_factory(config):
                robot = original_factory(config)
                profile.check_robot(robot)  # 两臂 connect() 之前验证标定/映射身份。
                return robot

            module.make_robot_from_config = checked_factory
        sys.argv = [saved_argv[0], *remaining]
        module.main()
    finally:
        module.teleop_loop = original
        if original_factory:
            module.make_robot_from_config = original_factory
        sys.argv = saved_argv
