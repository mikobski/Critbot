import React from "react";
import { Navbar, Badge } from "react-bootstrap";
import StatusConnection from "components/NavbarStatus/StatusConnection";

class NavbarStatus extends React.Component {
  render () {
    return (
      <div>
        <Navbar.Text>
          <StatusConnection/>
        </Navbar.Text>
      </div>
    );
  }
}

export default NavbarStatus;
