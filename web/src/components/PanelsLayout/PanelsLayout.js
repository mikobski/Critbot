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
    // const containerDim = this.panelsContainer.current.getBoundingClientRect();
    // const cameraPanelRatio = 16/9;
    // let cameraWidth = Math.ceil(containerDim.width/2);
    // let cameraHeight = Math.ceil(cameraWidth/cameraPanelRatio);
    // this.cameraPanel.current.style.height = `${cameraHeight}px`;
    // this.cameraPanel.current.style.width = `${cameraWidth}px`;
  };
  
  render() {
    // const keyCamera = "camera-0";
    // const keyMap = "map-0";
    // const elementMap = <Map key={ keyMap }/>;
    // let colLeft;
    // let colRightTop;
    // if(this.props.mode === Mode.MANUAL) {
    //   colLeft = <>          
    //       { elementCamera }       
    //       <div className="Panels-panel Panels-brd-top"
    //         style={{flexBasis: "50%" }}>
            
    //       </div>
    //     </>
    //   colRightTop = elementMap;
    // } else {
    //   colLeft = elementMap;
    //   colRightTop = elementCamera;
    // }

    return (
      <div ref={ this.panelsContainer } className="Panels-row">
        <div ref={ this.columnLeft } className="Panels-col Panels-brd-right" 
             style={{flexBasis: "50%" }}>
              <div className="Panels-panel" style={{flexGrow: 1}}>
                <Map/>
              </div>
              <div className="Panels-panel Panels-brd-top" style={{flexBasis: "350px"}}>
                <Measurements/>
              </div>
        </div>
        <div className="Panels-col" style={{flexBasis: "50%" }}>
          <div className="Panels-row" style={{ flexGrow: "1", height: "auto" }}>
            <div className="Panels-panel Panels-brd-right" style={{ flexBasis: "223px", padding: "1rem 1rem" }}>
              <StatusDetailed/>
              <EmergencyStop onModeChange={ this.props.onModeChange }/>
                {
                  this.props.mode === Mode.MANUAL && 
                  <ManualControl/> 
                }
            </div>
            <div className="Panels-panel" style={{ flexGrow: "1" }}>
              <Lidar physicalWidth="22" physicalHeight="22" gridMax="10" gridCount="4"/>
            </div>
          </div>
          <div className="Panels-panel Panels-brd-top" style={{ flexBasis: "540px" }} >
            <Camera/>
          </div>
        </div>
      </div>
    );
  }
};

export default PanelsLayout;