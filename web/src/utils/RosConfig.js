export const ROS_CONFIG = {
  defaultTopics: {
    camera: "/d400/color/image_raw/compressed",
    lidar: "/scan",
    manualControl: "/mavros/setpoint_velocity/cmd_vel_unstamped",
    modeSelect: "/critbot/mode_changes"
  }
};