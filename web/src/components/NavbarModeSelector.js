import React from 'react';
import { Button, ButtonGroup, Navbar } from 'react-bootstrap';

class NavbarModeSelector extends React.Component {
  render () {
    return (
      <Navbar.Collapse className="justify-content-end">
        <ButtonGroup>
            <Button variant="primary">Manual</Button>
            <Button variant="secondary">Autonomic</Button>
        </ButtonGroup>
      </Navbar.Collapse>
    );
  }
}

export default NavbarModeSelector;
