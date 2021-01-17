import React from 'react';
import { Navbar } from 'react-bootstrap';
import ModeSelector from './ModeSelector/ModeSelector';

class NavbarModeSelector extends React.Component {
  render () {
    return (
      <Navbar.Collapse className="justify-content-end">
        <ModeSelector {... this.props} />
      </Navbar.Collapse>
    );
  }
}

export default NavbarModeSelector;
