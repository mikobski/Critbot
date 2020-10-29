import './App.scss';
import AppLayout from './components/AppLayout'
import Navbar from 'react-bootstrap/Navbar';
import Container from 'react-bootstrap/Container';
import Row from 'react-bootstrap/Row';
import Col from 'react-bootstrap/Col';

function App() {
  return (
    <>
      <AppLayout
        nav={
          <Navbar bg="light">
            <Navbar.Brand href="#home">Critbot</Navbar.Brand>
          </Navbar>
        }
        content={
          <Container fluid>
            <Row>
              <Col>Test</Col>
              <Col>Test</Col>
            </Row>
          </Container>
        } />
    </>
  );
}

export default App;
