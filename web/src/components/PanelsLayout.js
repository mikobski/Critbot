import { Container, Row, Col } from "react-bootstrap";
import Camera from "components/Camera";
import Lidar from "components/Lidar";
import ManualControl from "components/ManualControl/ManualControl";
import EmergencyStop from "./EmergencyStop";
import StatusDetailed from "components/StatusDetailed/StatusDetailed";
import Map from "components/Map/Map";
import Measurements from "components/Measurements/Measurements";
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
          <StatusDetailed/>
          <EmergencyStop/>
          <ManualControl topic={ ROS_CONFIG.defaultTopics.manualControl } />
        </Col>
        <Col>
          <Map/>
        </Col>
      </Row>
      <Measurements/>
    </Container>
  );
};

export default PanelsLayout;