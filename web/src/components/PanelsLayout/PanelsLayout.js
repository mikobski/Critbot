import React from "react";
import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import EmergencyStop from "../EmergencyStop";
import StatusDetailed from "components/StatusDetailed/StatusDetailed";
import Map from "components/Map/Map";
import Measurements from "components/Measurements/Measurements";
import { ROS_CONFIG } from "utils/RosConfig";
import { MODE } from "utils/Mode";
import "components/PanelsLayout/PanelsLayout.scss";

class PanelsLayout extends React.Component {
  constructor(props) {
    super(props);
    this.panelsContainer = React.createRef();
  }

  render() {
    const StylesContainer = {
      height: "100%",
      display: "flex",
      flexDirection: "column"
    }

    return (
      <div ref={ this.panelsContainer } className="Panels Panels-container-col">
        <div className="Panels-col Panels-container">
          <div className="Panels-panel" style={{ 
              flexBasis: "50%",
              borderBottomWidth: ".15rem",
              borderRightWidth: ".15rem"
            }}>
            <Camera topic={ ROS_CONFIG.defaultTopics.camera }/> 
          </div>  
          <div className="Panels-panel" style={{ 
              flexBasis: "50%",
              flexGrow: "1",
              borderTopWidth: ".15rem",
              borderRightWidth: ".15rem"
            }}>
            <Lidar topic={ ROS_CONFIG.defaultTopics.lidar }
              physicalWidth="22" physicalHeight="22"
              gridMax="10" gridCount="4"/>
          </div>
        </div>
        <div className="Panels-col">
          <div style={{flexBasis: "50%"}} style={StylesContainer}>
            <div class="Panels-container-col">
              <div className="Panels-col Panels-container">
                <div className="Panels-panel" style={{ 
                  flexBasis: "50%",              
                  borderRightWidth: ".15rem",
                  borderBottomWidth: ".15rem",
                  borderLeftWidth: ".15rem",
                  padding: "0 1rem"
                }}>
                  <StatusDetailed/>
                  <EmergencyStop/>
                    {
                      this.props.mode == MODE.manual && 
                      <ManualControl topic={ ROS_CONFIG.defaultTopics.manualControl } /> 
                    }
                </div>
              </div>
              <div className="Panels-col">
                <div className="Panels-panel" style={{            
                    height: "100%",
                    borderLeftWidth: ".15rem",
                    borderBottomWidth: ".15rem"
                  }}>
                    <Map/>
                  </div>
                </div>
            </div>
            <div className="Panels-panel" style={{
              flexBasis: "100%",
              borderLeftWidth: ".15rem",
              borderTopWidth: ".15rem"
            }} >
              <Measurements/>
            </div>
          </div>
        </div>
      </div>
    );
  }
};

export default PanelsLayout;