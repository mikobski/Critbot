import './App.scss';

import VerticalContainer from './components/VerticalContainer/VerticalContainer';
import VerticalCol from './components/VerticalCol/VerticalCol';
import Navbar from 'react-bootstrap/Navbar';
import Container from 'react-bootstrap/Container';
import Row from 'react-bootstrap/Row';
import Col from 'react-bootstrap/Col';

function App() {
  return (
      <VerticalContainer>
        <VerticalCol basis="auto">
          <Navbar bg="light">
            <Navbar.Brand href="#home">Critbot</Navbar.Brand>
            <StatusShortened/>
          </Navbar>
        </VerticalCol>
        <VerticalCol basis="100%">
          <Container fluid>
            <Row>
              <Col>Test</Col>
              <Col>Test</Col>
            </Row>
          </Container>
        </VerticalCol>
      </VerticalContainer>
  );
}

export default App;
