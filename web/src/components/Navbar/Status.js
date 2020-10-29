import React from "react";
import { Navbar, Badge } from "react-bootstrap";
import StatusConnection from "./StatusConnection";

class Status extends React.Component {
  render () {
    console.log(this.props.ros);
    return (
      <div>
        <Navbar.Text>
          <StatusConnection ros={ this.props.ros }/>
        </Navbar.Text>
        <Navbar.Text style={{marginLeft: "5px"}}>
          <span>Battery: </span>
          <Badge variant="success">100%</Badge>
        </Navbar.Text>
        <Navbar.Text style={{marginLeft: "5px"}}>
          <span>Radio: </span>
          <Badge variant="success">100%</Badge>
        </Navbar.Text>
      </div>
    );
  }
}

export default Status;
