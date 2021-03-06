import React from "react";
import { Button } from "react-bootstrap";
import { ArrowClockwise, ArrowCounterclockwise, ArrowDown, ArrowUp } from "react-bootstrap-icons";
import { Direction } from "components/ManualControl/Direction";

class ButtonControl extends React.Component {

  handlePull = () => {
    this.props.onPull(this.props.dir) 
  };
  render () {
    const stylesBtn = {
      margin: "3px",
      width: "56px",
      height: "56px",
      padding: "0"
    }
    const iconSize = 36;
    let icon;
    if(this.props.dir === Direction.FORWARD) {
      icon = <ArrowUp size={ iconSize }/>;
    } else if(this.props.dir === Direction.BACKWARD) {
      icon = <ArrowDown size={ iconSize }/>;
    } else if(this.props.dir === Direction.LEFT) {
      icon = <ArrowCounterclockwise size={ iconSize }/>;
    } else if(this.props.dir === Direction.RIGHT) {
      icon = <ArrowClockwise size={ iconSize }/>;
    }
    return (
      <Button variant="primary" size="lg" style={ stylesBtn } 
        active={ this.props.active }
        className={ this.props.active ? "focus" : null }
        onMouseDown={ (e) => { 
          this.props.onPush(this.props.dir);
          e.preventDefault();
        }}
        onDrag={ (e) => { 
          e.preventDefault();
        }}
        onMouseLeave={ this.handlePull }
        onMouseUp={ this.handlePull }
        onBlur={ this.handlePull }>        
        { icon }
      </Button>
    );
  }
}

export default ButtonControl;
