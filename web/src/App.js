import React from "react";
import AppLayout from "components/AppLayout";
import RosClient from "RosClient/RosLibJsClient";
import { RosContext } from "utils/RosContext";
import "App.scss";


class App extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      rosClient: new RosClient({
        url: "ws://localhost:9090",
        // url: "ws://192.168.1.161:9090"
      })
    }
  }

  componentDidMount() {
    this.state.rosClient.connection.connect();
  }
  componentWillUnmount() {
    this.state.rosClient.connection.close();
  }

  render() {
    return (
      <RosContext.Provider value= { this.state.rosClient }>
        <AppLayout/>
      </RosContext.Provider>
    );
  }
}

export default App;
