export const ROS_CONFIG = {
  defaultTopics: {
    camera: "/d400/color/image_raw/compressed",
    lidar: "/scan",
    manualControl: "/critbot/manual_control",
    modeSelect: "/critbot/mode_changes",
    statusBattery: "/mavros/battery",
    statusWifi: "/wifi_status"
  },
  defaultURL: "ws://localhost:9090"
  // defaultURL: "ws://192.168.1.161:9090"
  // defaultURL: "ws://rosnuc:9090"
};