import React from "react";
import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import EmergencyStop from "../EmergencyStop";
import StatusDetailed from "components/StatusDetailed/StatusDetailed";
import Map from "components/Map/Map";
import Measurements from "components/Measurements/Measurements";
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
  };
  
  render() {
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