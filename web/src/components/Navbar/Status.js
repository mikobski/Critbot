import React from 'react';
import { Navbar, Badge } from 'react-bootstrap';

class Status extends React.Component {
  render () {
    return (
      <div>
        <Navbar.Text>
          <span>Connection: </span>
          <Badge variant="success">OK <i>(localhost)</i></Badge>
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
