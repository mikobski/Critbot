import "App.scss";
import React from "react";
import RosClient from "RosClient/RosLibJsClient";

import StatusNavbar from "components/Navbar/Status";
import ModeSelectorNavbar from "components/Navbar/ModeSelector";
import VerticalContainer from "helpers/VerticalContainer";
import VerticalCol from "helpers/VerticalRow";
import { Navbar } from "react-bootstrap";
import PanelsLayout from "components/PanelsLayout";

//const ROS_URI = "ws://192.168.1.161:9090";
const ROS_URI = "ws://localhost:9090";

const rosClient = new RosClient({
  url: ROS_URI
});

class App extends React.Component {
  constructor(props) {
    super(props)
    this.rosClient = rosClient;
  }
  componentDidMount() {
    this.rosClient.connection.connect();
  }
  componentWillUnmount() {
    this.rosClient.connection.close();
  }

  render() {
    return (
        <VerticalContainer>
          <VerticalCol basis="auto">
            <Navbar bg="dark" variant="dark">
              <Navbar.Brand>Critbot</Navbar.Brand>
              <StatusNavbar ros={ this.rosClient }/>
              <ModeSelectorNavbar/>
            </Navbar>
          </VerticalCol>
          <VerticalCol basis="100%">
            <PanelsLayout/>
          </VerticalCol>
        </VerticalContainer>
    );
  }
}

export default App;
