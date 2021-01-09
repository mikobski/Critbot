import React from "react";
import ButtonControl from "components/ManualControl/ButtonControl";
import { Direction } from "components/ManualControl/Direction";
import { RosContext } from "utils/RosContext";
import RosTopic from "RosClient/Topic";
import { Form } from "react-bootstrap";

class ManualControl extends React.PureComponent {
  static contextType = RosContext;
  static defaultProps = {
    repeatingTime: 300,
    moveSpeed: 1,
    rotateSpeed: 1,
    speedPercentMin: 10,
    speedPercentMax: 100,
    speedPercentStep: 5,
  };

  _topic;
  _intervalHandler;

  constructor(props) {
    super(props);
    this.state = {
      direction: Direction.STOP,
      speedPercent: this.props.speedPercentMax
    }
    this.refSlider = React.createRef();
  }
  componentDidMount(){
    const rosClient = this.context;
    const topicName = this.props.topic;
    this._topic = new RosTopic({
      ros: rosClient,
      name: topicName,
      messageType: "geometry_msgs/Twist", 
    });
    this._topic.advertise();
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
      direction: Direction.STOP
    });
  };
  handleKeyDown = (e) => {
    let dir = null;
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
      direction: Direction.STOP
    });
  }

  _moveCmd = () => {
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
    const moveSpeed = this.props.moveSpeed*this.state.speedPercent/100;
    const rotateSpeed = this.props.rotateSpeed*this.state.speedPercent/100;
    if(direction === Direction.FORWARD) {
      msg.linear.y = moveSpeed;
    } else if(direction === Direction.BACKWARD) {
      msg.linear.y = -moveSpeed;
    } else if(direction === Direction.LEFT) {
      msg.angular.z = rotateSpeed;
    } else if(direction === Direction.RIGHT) {
      msg.angular.z = -rotateSpeed;
    }
    this._topic.publish(msg);
  };

  handlePercenteChange = (e) => {
    this.setState({
      speedPercent: e.target.value
    });
    this.refSlider.current.blur();
  };

  componentDidUpdate(prevProps, prevState) {
    const { direction } = this.state;
    if(direction === Direction.STOP) {
      if(this._intervalHandler != null) {
        clearInterval(this._intervalHandler);
        this._intervalHandler = null;
        this._moveCmd(direction)
      }
    } else if(prevState !== this.state) {
      if(this._intervalHandler == null) {
        this._moveCmd(direction);
      } else {
        clearInterval(this._intervalHandler);
      }
      this._intervalHandler = setInterval(() => {
        this._moveCmd(direction);
      }, this.props.repeatingTime);
    }
  }

  render () {
    const { direction } = this.state;
    const stylesContainer = {
      display: "flex",
      padding: "1rem 0",
      flexDirection: "column",
      justifyContent: "center"
    };
    const stylesRow = {
      display: "flex",
      justifyContent: "center"
    };
    return (
      <div style={ stylesContainer }>
        <div style={{ ...stylesRow, justifyContent: "space-between" }}>
          <div className="text-muted"><small>10%</small></div>
          <div className="text-primary">{ this.state.speedPercent }%</div>
          <div className="text-muted"><small>100%</small></div>
        </div>
        <div style={{ padding: "0 0 1rem 0", ...stylesRow }}>
          <Form.Control type="range" ref={ this.refSlider }
            value={ this.state.speedPercent } onChange={ this.handlePercenteChange }
            min={ this.props.speedPercentMin } max={ this.props.speedPercentMax }
            step={ this.props.speedPercentStep }/>
        </div>
        <div style={ stylesRow }>
          <ButtonControl 
            onStartCmd={ this.handleStartCmd } 
            onStopCmd={ this.handleStopCmd } 
            dir={ Direction.FORWARD }
            active={ direction === Direction.FORWARD }/>
        </div>
        <div style={ stylesRow }>
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

export default ManualControl;
