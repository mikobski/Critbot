import React from 'react';
import { Button } from 'react-bootstrap';

class ButtonMode extends React.Component {
  _refBtn;
  constructor(props) {
    super(props);
    this._refBtn = React.createRef();
  }
  handleBtnClick = (e) => {
    this.props.onModeChange(this.props.mode, this._refBtn.current);
  };
  render () {
    return (
        <Button variant={ this.props.active ? 'primary': 'secondary' }
          onClick={ this.handleBtnClick }
          ref={ this._refBtn }>
          { this.props.children }
        </Button>
    );
  }
}

export default ButtonMode;