import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import { Container, Row, Col } from "react-bootstrap";
import VerticalContainer from "helpers/VerticalContainer";
import VerticalCol from "helpers/VerticalRow";
import { ROS_CONFIG } from "utils/RosConfig";

const PanelsLayout = (props) => {
  return (
    <Container fluid>
      <Row>
        <Col>
          <VerticalContainer>
            <VerticalCol basis="50%">
            <Camera topic={ ROS_CONFIG.defaultTopics.camera }/> 
            </VerticalCol>
            <VerticalCol basis="50%">
              <Lidar topic={ ROS_CONFIG.defaultTopics.lidar }
                physicalWidth="22" physicalHeight="22"
                gridMax="10" gridCount="4"/>
            </VerticalCol>
          </VerticalContainer>
        </Col>
        <Col>
          
          <ManualControl topic={ ROS_CONFIG.defaultTopics.manualControl } />
        </Col>
      </Row>
    </Container>
  );
};

export default PanelsLayout;