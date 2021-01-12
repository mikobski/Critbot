export const ROS_CONFIG = {
  defaultTopics: {
    camera: "/d400/color/image_raw/compressed",
    lidar: "/scan",
    manualControl: "/critbot/manual_control",
    modeSelect: "/critbot/mode_changes",
    battery: "/mavros/battery",
    wifi: "/wifi_status"
  }
};