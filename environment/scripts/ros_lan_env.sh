# 上位机 192.168.2.1 与下位机 192.168.2.230 共用这一套 ROS 通信环境。
# 调用前要先设置 SDK_ROOT。
if [[ -z "${SDK_ROOT:-}" ]]; then
  echo "ros_lan_env.sh 需要 SDK_ROOT" >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${SDK_ROOT}/environment/scripts/fastdds_localhost.xml"
