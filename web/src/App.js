import React from "react";
import AppLayout from "components/AppLayout";
import RosClient from "RosClient";
import { RosContext } from "utils/RosContext";
import "App.scss";
import { Mode } from "utils/Mode";


class App extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      rosClient: new RosClient({
        reconnectTimeout: 3000
      }),
      mode: Mode.DISARMED
    };
    this.state.rosClient.on("error", () => {
      console.info(`[Critbot] Websocket connection to '${this.state.rosClient.socket.url}' is refused`);
    });
  }

  componentDidMount() {
    const rosURL = "ws://localhost:9090";
    // const rosURL = "ws://192.168.1.161:9090";
    this.state.rosClient.connect(rosURL);
  }
  componentWillUnmount() {
    this.state.rosClient.close();
  }

  handleModeChanged = (mode, btn) => {
    setTimeout(() => {
      this.setState({
        mode: mode
      });
      btn.blur();
    }, 500);
  };

  render() {
    return (
      <RosContext.Provider value={ this.state.rosClient }>
        {
         <AppLayout mode={ this.state.mode } onModeChange={ this.handleModeChanged }/> 
        }
      </RosContext.Provider>
    );
  }
}

export default App;
