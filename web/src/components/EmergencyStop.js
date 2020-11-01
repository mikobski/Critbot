import React from "react";
import { Button } from "react-bootstrap";

class EmergencyStop extends React.Component {
  render() {
    return (
      <div>
        <Button variant="danger" size="lg" block>STOP</Button>
      </div>
    );
  }
}

export default EmergencyStop;