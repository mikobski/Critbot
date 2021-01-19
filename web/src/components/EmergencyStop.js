import React from "react";
import { Button } from "react-bootstrap";
import { Mode } from "utils/Mode";

class EmergencyStop extends React.Component {
  _refBtn;
  constructor(props) {
    super(props);
    this._refBtn = React.createRef();
  }
  handleClick = () => {
    this.props.onModeChange(Mode.DISARMED, this._refBtn.current);
  };
  render() {
    return (
      <div>
        <Button variant="danger" size="lg" block
        onClick={ this.handleClick }
        ref={ this._refBtn }>STOP</Button>
      </div>
    );
  }
}

export default EmergencyStop;