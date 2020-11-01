import React from "react";
import ButtonControl from "components/ManualControl/ButtonControl";
import { Direction } from "components/ManualControl/Direction";
import { RosContext } from "utils/RosContext";

class ManualControl extends React.PureComponent {
  static contextType = RosContext;
  _intervalHandler;

  constructor(props) {
    super(props);
    this.state = {
      direction: null
    }
  }
  componentDidMount(){
    document.addEventListener("keydown", this.handleKeyDown, false);
    document.addEventListener("keyup", this.handleKeyUp, false);
  }
  componentWillUnmount(){
    document.removeEventListener("keydown", this.handleKeyDown, false);
    document.removeEventListener("keyup", this.handleKeyUp, false);
  }

  handleStartCmd = (dir) => {
    this.setState({
      direction: dir
    });
  };
  handleStopCmd = (dir) => {
    this.setState({
      direction: null
    });
  };
  handleKeyDown = (e) => {
    let dir;
    if(e.code === "ArrowUp") {
      dir = Direction.FORWARD;
    } else if(e.code === "ArrowDown") {
      dir = Direction.BACKWARD;
    } else if(e.code === "ArrowLeft") {
      dir = Direction.LEFT;
    } else if(e.code === "ArrowRight") {
      dir = Direction.RIGHT;
    }
    this.setState({
      direction: dir
    });
  }
  handleKeyUp = (e) => {
    this.setState({
      direction: null
    });
  }

  _moveCmd = () => {
    const messageType = "geometry_msgs/Twist"
    const { direction } = this.state;
    let msg = {
      "linear": {
        "x": 0,
        "y": 0,
        "z": 0
      },
      "angular": {
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
    if(direction === Direction.FORWARD) {
      msg.linear.x = this.props.moveStep;
    } else if(direction === Direction.BACKWARD) {
      msg.linear.x = -this.props.moveStep;
    } else if(direction === Direction.LEFT) {
      msg.angular.z = this.props.rotateStep;
    } else if(direction === Direction.RIGHT) {
      msg.angular.z = -this.props.rotateStep;
    }
    this.context.topic.publish(this.props.topic, messageType, msg);
  };

  componentDidUpdate() {
    const { direction } = this.state;
    if(direction == null) {
      if(this._intervalHandler != null) {
        clearInterval(this._intervalHandler);
        this._intervalHandler = null;
      }
    } else {
      if(this._intervalHandler == null) {
        this._moveCmd(direction);
      } else {
        clearInterval(this._intervalHandler);
      }
      this._intervalHandler = setInterval(() => {
        this._moveCmd(direction)
      }, this.props.repeatingTime);
    }
  }

  render () {
    const { direction } = this.state;
    const stylesContainer = {
      display: "flex",
      height: "100%",
      paddingTop: "15px",
      paddingBottom: "15px",
      flexDirection: "column",
      justifyContent: "center",
      textAlign: "center"
    };
    return (
      <div style={ stylesContainer }>
        <div>
          <ButtonControl 
            onStartCmd={ this.handleStartCmd } 
            onStopCmd={ this.handleStopCmd } 
            dir={ Direction.FORWARD }
            active={ direction === Direction.FORWARD }/>
        </div>
        <div>
          <ButtonControl 
            onStartCmd={ this.handleStartCmd } 
            onStopCmd={ this.handleStopCmd }  
            dir={ Direction.LEFT }
            active={ direction === Direction.LEFT }/>
          <ButtonControl 
            onStartCmd={ this.handleStartCmd } 
            onStopCmd={ this.handleStopCmd }  
            dir={ Direction.BACKWARD }
            active={ direction === Direction.BACKWARD }/>
          <ButtonControl 
            onStartCmd={ this.handleStartCmd } 
            onStopCmd={ this.handleStopCmd }  
            dir={ Direction.RIGHT }
            active={ direction === Direction.RIGHT }/>
        </div>
      </div>
    );
  }
}
ManualControl.defaultProps = {
  repeatingTime: 300,
  moveStep: 1,
  rotateStep: 1
};

export default ManualControl;
