"""工具脚本共用的 ROS 环境自检。

键盘脚本里有同样一段。start_upper_moveit.sh 会 source ros_lan_env.sh，
但单独跑 python3 tools/xxx.py 时不会，于是节点跑到默认域 0 上，
表现就是「收不到 /joint_states」「服务等不到」。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

DEFAULT_DOMAIN = "3"


def find_dds_profile() -> Optional[Path]:
    here = Path(__file__).resolve()
    candidates = [parent / "environment" / "scripts" / "fastdds_localhost.xml"
                  for parent in here.parents]
    sdk_root = os.environ.get("SDK_ROOT", "")
    if sdk_root:
        candidates.insert(0, Path(sdk_root) / "environment" / "scripts" / "fastdds_localhost.xml")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def ensure_ros_env() -> None:
    """在 rclpy.init 之前调用；已经设好的环境不覆盖。"""
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if domain != DEFAULT_DOMAIN:
        os.environ["ROS_DOMAIN_ID"] = DEFAULT_DOMAIN
        print(f"[x1] ROS_DOMAIN_ID={domain or '(未设置)'} → {DEFAULT_DOMAIN}，与下位机同域。", flush=True)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        profile = find_dds_profile()
        if profile is not None:
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
            print(f"[x1] DDS 配置 {profile}", flush=True)
        else:
            print("[x1] 未找到 fastdds_localhost.xml，用默认 DDS 发现。", flush=True)
