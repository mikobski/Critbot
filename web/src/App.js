import React from "react";
import AppLayout from "components/AppLayout";
import RosClient from "RosClient";
import Service from "RosClient/Service";
import { RosContext } from "utils/RosContext";
import "App.scss";
import { Mode } from "utils/Mode";
import { ROS_CONFIG } from "utils/RosConfig";


class App extends React.Component {
  _modeSelectService;
  _rosClient;
  constructor(props) {
    super(props);
    this._rosClient = new RosClient({
      reconnectTimeout: 3000
    });
    this.state = {
      mode: Mode.DISARMED
    };
    this._rosClient.on("error", () => {
      console.info(`[Critbot] Websocket connection to '${this._rosClient.socket.url}' is refused`);
    });
    this._rosClient.on("connection", () => {
      this.handleModeChanged(Mode.DISARMED);
    });
  }

  componentDidMount() {
    // const rosURL = "ws://localhost:9090";
    // const rosURL = "ws://192.168.1.161:9090";
    const rosURL = "ws://rosnuc:9090";
    this._modeSelectService = new Service({
      ros: this._rosClient,
      name: ROS_CONFIG.defaultTopics.modeSelect,
      type: "ModeChanges"
    });
    this._rosClient.connect(rosURL);
  }
  componentWillUnmount() {
    this._rosClient.close();
  }

  handleModeChanged = (mode, btn) => {
    let rosMode = "emergency_stop";
    if(mode == Mode.MANUAL) {
      rosMode = "manual";
    } else if(mode == Mode.AUTO) {
      rosMode = "autonomic";
    }
    this._modeSelectService.callService([rosMode], (msg, btn) => {
      let mode = Mode.DISARMED;
      if(msg) {
        if(msg.message == "emergency_stop") {
          mode = Mode.DISARMED;
        } else if(msg.message == "autonomic") {
          mode = Mode.AUTO;
        } else if(msg.message == "manual") {
          mode = Mode.MANUAL;
        } else {
          console.error("Changing mode (to '"+mode+"') failed!");
        }
      } else {
        console.error("Changing mode (to '"+mode+"') failed!");
      }
      
      this.setState({
        mode: mode
      });
      if(btn) {
        btn.blur();
      }
    }, function() {
      console.error("Changing mode (to '"+mode+"') failed!");
    });
  };

  render() {
    return (
      <RosContext.Provider value={ this._rosClient }>
        {
         <AppLayout mode={ this.state.mode } onModeChange={ this.handleModeChanged }/> 
        }
      </RosContext.Provider>
    );
  }
}

export default App;
