import './App.scss';

import StatusNavbar from './components/Navbar/Status';
import ModeSelectorNavbar from './components/Navbar/ModeSelector';
import Camera from './components/Panel/Camera';
import Lidar from './components/Panel/Lidar';
import ManualControl from './components/Panel/ManualControl';

import VerticalContainer from './helpers/VerticalContainer';
import VerticalCol from './helpers/VerticalRow';
import { Navbar } from 'react-bootstrap';
import { Container, Row, Col } from 'react-bootstrap';

function App() {
  return (
      <VerticalContainer>
        <VerticalCol basis="auto">
          <Navbar bg="dark" variant="dark">
            <Navbar.Brand>Critbot</Navbar.Brand>
            <StatusNavbar/>
            <ModeSelectorNavbar/>
          </Navbar>
        </VerticalCol>
        <VerticalCol basis="100%">
          <Container fluid>
            <Row>
              <Col>
                <VerticalContainer>
                  <VerticalCol basis="50%">
                    <Camera/>
                  </VerticalCol>
                  <VerticalCol basis="50%">
                    <Lidar/>
                  </VerticalCol>
                </VerticalContainer>
              </Col>
              <Col>
                <ManualControl/>
              </Col>
            </Row>
          </Container>
        </VerticalCol>
      </VerticalContainer>
  );
}

export default App;
