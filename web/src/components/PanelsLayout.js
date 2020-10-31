import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import { Container, Row, Col } from "react-bootstrap";
import VerticalContainer from "helpers/VerticalContainer";
import VerticalCol from "helpers/VerticalRow";

const PanelsLayout = (props) => {
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
};

export default PanelsLayout;