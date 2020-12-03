import React from 'react';
import { ButtonGroup } from 'react-bootstrap';
import { Mode } from 'utils/Mode';
import ButtonMode from './ButtonMode';

class ModeSelector extends React.Component {
  render () {
    return (
      <ButtonGroup>
        <ButtonMode mode={ Mode.AUTO }
          active={ this.props.mode === Mode.AUTO } 
          onModeChange={ this.props.onModeChange } >
            Autonomic
        </ButtonMode>
        <ButtonMode mode={ Mode.MANUAL }
          active={ this.props.mode === Mode.MANUAL }
          onModeChange={ this.props.onModeChange } >
            Manual
        </ButtonMode>
        <ButtonMode mode={ Mode.DISARMED }
          active={ this.props.mode === Mode.DISARMED }
          onModeChange={ this.props.onModeChange } >
            Disarmed
        </ButtonMode>
      </ButtonGroup>
    );
  }
}

export default ModeSelector;
