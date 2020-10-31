import "./App.scss";
import React from "react";
import RosClient from "./RosClient/RosLibJsClient";

import StatusNavbar from "./components/Navbar/Status";
import ModeSelectorNavbar from "./components/Navbar/ModeSelector";
import Camera from "./components/Camera";
import Lidar from "./components/Lidar";
import ManualControl from "./components/ManualControl/ManualControl";

import VerticalContainer from "./helpers/VerticalContainer";
import VerticalCol from "./helpers/VerticalRow";
import { Navbar } from "react-bootstrap";
import { Container, Row, Col } from "react-bootstrap";

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
            <Container fluid>
              <Row>
                <Col>
                  <VerticalContainer>
                    <VerticalCol basis="50%">
                      <Camera ros={ this.rosClient } topic="/d400/color/image_raw/compressed"/>
                    </VerticalCol>
                    <VerticalCol basis="50%">
                      <Lidar ros={ this.rosClient } topic="/scan"
                        physicalWidth="22" physicalHeight="22"
                        gridMax="10" gridCount="4"/>
                    </VerticalCol>
                  </VerticalContainer>
                </Col>
                <Col>
                  
                  <ManualControl ros={ this.rosClient } topic="/turtle1/cmd_vel" />
                </Col>
              </Row>
            </Container>
          </VerticalCol>
        </VerticalContainer>
    );
  }
}

export default App;
