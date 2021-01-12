import React from "react";
import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import EmergencyStop from "../EmergencyStop";
import StatusDetailed from "components/StatusDetailed/StatusDetailed";
import Map from "components/Map/Map";
import Measurements from "components/Measurements/Measurements";
import { ROS_CONFIG } from "utils/RosConfig";
import { Mode } from "utils/Mode";
import "components/PanelsLayout/PanelsLayout.scss";

class PanelsLayout extends React.Component {
  constructor(props) {
    super(props);
    this.panelsContainer = React.createRef();
    this.columnLeft = React.createRef();
    this.cameraPanel = React.createRef();
    this.columnRightTop = React.createRef();
  }

  componentDidMount() {
    this.resizeLayout();
    window.addEventListener("resize", this.resizeLayout);
  }
  componentDidUpdate() {
    this.resizeLayout();    
  }
  componentWillUnmount() {
    window.removeEventListener("resize", this.resizeLayout);
  }

  resizeLayout = () => {
    const containerDim = this.panelsContainer.current.getBoundingClientRect();
    const cameraPanelRatio = 16/9;
    let cameraWidth = Math.ceil(containerDim.width/2);
    let cameraHeight = Math.ceil(cameraWidth/cameraPanelRatio);
    this.cameraPanel.current.style.height = `${cameraHeight}px`;
    this.cameraPanel.current.style.width = `${cameraWidth}px`;
  };
  
  render() {
    const keyCamera = "camera-0";
    const keyMap = "map-0";
    const elementMap = <Map key={ keyMap }/>;
    const elementCamera = <div ref={ this.cameraPanel } className="Panels-panel" >
        <Camera key={ keyCamera } topic={ ROS_CONFIG.defaultTopics.camera }/>;
      </div>
    let colLeft;
    let colRightTop;
    if(this.props.mode === Mode.MANUAL) {
      colLeft = <>          
          { elementCamera }       
          <div className="Panels-panel Panels-brd-top"
            style={{flexBasis: "50%" }}>
            <Lidar topic={ ROS_CONFIG.defaultTopics.lidar }
              physicalWidth="22" physicalHeight="22" gridMax="10" gridCount="4"/>
          </div>
        </>
      colRightTop = elementMap;
    } else {
      colLeft = elementMap;
      colRightTop = elementCamera;
    }

    return (
      <div ref={ this.panelsContainer } className="Panels Panels-container-col">
        <div ref={ this.columnLeft } className="Panels-col Panels-container Panels-brd-right" 
             style={{flexBasis: "50%" }}>
            { colLeft }
        </div>
        <div className="Panels-col" style={{flexBasis: "50%" }}>
          <div style={{flexBasis: "50%" }} className="Panels-container">
            <div className="Panels-container-col">
              <div className="Panels-col Panels-container" style={{ flexBasis: "0px" }}>
                <div className="Panels-panel Panels-container Panels-brd-right" style={{ flexBasis: "100%", padding: "0 1rem" }}>
                  <StatusDetailed topicBattery={ ROS_CONFIG.defaultTopics.battery }
                    topicWifi={ ROS_CONFIG.defaultTopics.wifi }/>
                  <EmergencyStop onModeChange={ this.props.onModeChange }/>
                    {
                      this.props.mode === Mode.MANUAL && 
                      <ManualControl topic={ ROS_CONFIG.defaultTopics.manualControl } /> 
                    }
                </div>
              </div>
              <div className="Panels-col" style={{ flexBasis: "100%" }}>
                <div ref={ this.columnRightTop }className="Panels-panel" style={{ height: "100%" }}>
                    { colRightTop }
                </div>
              </div>
            </div>
            <div className="Panels-panel Panels-brd-top" style={{ flexBasis: "100%" }} >
              <Measurements/>
            </div>
          </div>
        </div>
      </div>
    );
  }
};

export default PanelsLayout;