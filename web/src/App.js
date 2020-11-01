import "App.scss";
import React from "react";
import RosClient from "RosClient/RosLibJsClient";
import { RosContext } from "utils/RosContext";

import StatusNavbar from "components/Navbar/Status";
import ModeSelectorNavbar from "components/Navbar/ModeSelector";
import VerticalContainer from "helpers/VerticalContainer";
import VerticalCol from "helpers/VerticalRow";
import { Navbar } from "react-bootstrap";
import PanelsLayout from "components/PanelsLayout";

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
        <VerticalContainer>
          <VerticalCol basis="auto">
            <Navbar bg="dark" variant="dark">
              <Navbar.Brand>Critbot</Navbar.Brand>
              <StatusNavbar/>
              <ModeSelectorNavbar/>
            </Navbar>
          </VerticalCol>
          <VerticalCol basis="100%">
            <PanelsLayout/>
          </VerticalCol>
        </VerticalContainer>
      </RosContext.Provider>
    );
  }
}

export default App;
