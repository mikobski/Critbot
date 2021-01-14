import React from "react";
import ButtonControl from "components/ManualControl/ButtonControl";
import { Direction } from "components/ManualControl/Direction";
import { RosContext } from "utils/RosContext";
import RosTopic from "RosClient/Topic";
import { Form } from "react-bootstrap";
import { ROS_CONFIG } from "utils/RosConfig";
import DirectionsMap from "./DirectionsMap";

class ManualControl extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    repeatingTime: 300,
    moveSpeed: 1,
    rotateSpeed: 1,
    speedPercentMin: 10,
    speedPercentMax: 100,
    speedPercentStep: 5,
    topic: ROS_CONFIG.defaultTopics.manualControl
  };
  _topic;
  _btnsDirs;
  _keyDirs;
  _drivingVel;
  _stop;
  _timeoutHandler = null;

  constructor(props) {
    super(props);
    this.state = {
      drivingDirs: new DirectionsMap(),
      speedPercent: this.props.speedPercentMax,
    }
    this.refSlider = React.createRef();
    this._btnsDirs = new DirectionsMap();
    this._keyDirs = new DirectionsMap();
    this._drivingVel = {
      lin: 0,
      ang: 0
    };
    this._stop = true;
  }
  componentDidMount(){
    const rosClient = this.context;
    this._topic = new RosTopic({
      ros: rosClient,
      name: this.props.topic,
      messageType: "geometry_msgs/Twist", 
    });
    this._topic.advertise();
    document.addEventListener("keydown", this.handleKeyDown, false);
    document.addEventListener("keyup", this.handleKeyUp, false);
    window.addEventListener("blur", this.handleClearAndStop);
  }
  componentWillUnmount(){
    this._topic.unadvertise();
    if(this._timeoutHandler !== null) {
      clearTimeout(this._timeoutHandler);
    }
    document.removeEventListener("keydown", this.handleKeyDown, false);
    document.removeEventListener("keyup", this.handleKeyUp, false);
    window.removeEventListener("blur", this.handleClearAndStop);
  }
  handleClearAndStop = () => {
    this._btnsDirs.clearAll();
    this._keyDirs.clearAll();
    this._updateDrivingVel();
  };
  _keyCodeToDir(code) {
    if(code === "ArrowUp")
      return Direction.FORWARD;
    if(code === "ArrowDown")
      return Direction.BACKWARD;
    if(code === "ArrowLeft")
      return Direction.LEFT;
    if(code === "ArrowRight")
      return Direction.RIGHT;
    return null;
  }
  handleKeyDown = (e) => {
    let dir = this._keyCodeToDir(e.code);
    if(dir) {
      this._keyDirs.setDir(dir);
      this._updateDrivingVel();
    }
  }
  handleKeyUp = (e) => {
    let dir = this._keyCodeToDir(e.code);
    if(dir) {
      this._keyDirs.clearDir(dir);
      this._updateDrivingVel();
    }
  }
  handleBtnPush = (dir) => {
    this._btnsDirs.setDir(dir);
    this._updateDrivingVel();
  };
  handleBtnPull = (dir) => {
    this._btnsDirs.clearDir(dir);
    this._updateDrivingVel();
  };

  _updateDrivingVel() {
    let vel = {lin: 0, ang: 0};
    let stop = true;
    if(this._keyDirs.anyDir()) {
      vel = this._dirsToVel(this._keyDirs);
      stop = false;
    } else if(this._btnsDirs.anyDir()) {
      vel = this._dirsToVel(this._btnsDirs);
      stop = false;
    }
    if(!(this._stop && stop)) {
      this._stop = stop;
      this._drivingVel = vel;
      this.moveCmd();
    }

    let newDirs = new DirectionsMap();
    if(!stop) {
      newDirs = this._velToDirs(vel);
    }
    this.setState({
      drivingDirs: newDirs
    });
  };
  moveCmd = () => {
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
    if(this._timeoutHandler !== null) {
      clearTimeout(this._timeoutHandler)
    }
    if(!this._stop) {
      msg.linear.y = this._drivingVel.lin*moveSpeed;
      msg.angular.z = this._drivingVel.ang*rotateSpeed;    
      this._timeoutHandler = setTimeout(this.moveCmd, this.props.repeatingTime);
    }
    this._topic.publish(msg);
  };
  _dirsToVel(dirs) {
    let vel = { lin: 0, ang: 0 };
    if(dirs.getDir(Direction.FORWARD))
      vel.lin += 1;
    if(dirs.getDir(Direction.BACKWARD))
      vel.lin -= 1;
    if(dirs.getDir(Direction.RIGHT))
      vel.ang += 1;
    if(dirs.getDir(Direction.LEFT))
      vel.ang -= 1;
    return vel;
  }
  _velToDirs(vel) {
    let dirs = new DirectionsMap();
    if(vel.lin > 0) {
      dirs.setDir(Direction.FORWARD);
    } else if(vel.lin < 0) {
      dirs.setDir(Direction.BACKWARD);
    }
    if(vel.ang > 0) {
      dirs.setDir(Direction.RIGHT);      
    } else if(vel.ang < 0) {
      dirs.setDir(Direction.LEFT);
    }
    return dirs;
  }
  handlePercenteChange = (e) => {
    this.setState({
      speedPercent: e.target.value
    });
    this.refSlider.current.blur();
  };
  shouldComponentUpdate(nextProps, nextState) {
    return this.state.speedPercent !== nextState.speedPercent ||
      !this.state.drivingDirs.compare(nextState.drivingDirs);
  }
  render () {
    const showedDirs = this.state.drivingDirs;
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
            onPush={ this.handleBtnPush } 
            onPull={ this.handleBtnPull } 
            dir={ Direction.FORWARD }
            active={ showedDirs.getDir(Direction.FORWARD)}/>
          </div>
          <div style={ stylesRow }>
          <ButtonControl 
            onPush={ this.handleBtnPush } 
            onPull={ this.handleBtnPull } 
            dir={ Direction.LEFT }
            active={ showedDirs.getDir(Direction.LEFT)}/>
          <ButtonControl 
            onPush={ this.handleBtnPush } 
            onPull={ this.handleBtnPull } 
            dir={ Direction.BACKWARD }
            active={ showedDirs.getDir(Direction.BACKWARD)}/>
          <ButtonControl 
            onPush={ this.handleBtnPush } 
            onPull={ this.handleBtnPull } 
            dir={ Direction.RIGHT }
            active={ showedDirs.getDir(Direction.RIGHT)}/>
        </div>
      </div>
    );
  }
}

export default ManualControl;
