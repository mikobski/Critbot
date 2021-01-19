import React from "react";
import CanvasWithStatus from "helpers/CanvasWithStatus";
import { RosContext } from "utils/RosContext";

class CanvasDataFromRos extends React.PureComponent {
  static contextType = RosContext;
  _data = null;

  constructor(props) {
    super(props);
    this.state = {
      dataValid: false
    };
  }

  componentDidMount() {
    this.props.topic.subscribe(this.topicListener, this.errorListener);
  }
  componentWillUnmount() {
    this.props.topic.unsubscribe(this.topicListener, this.errorListener);
  }
  topicListener = (message) => {
    this.props.promiseNewData(message).then((data) => {
      this._data = data;
      this.setState({
        dataValid: true
      });
    }).catch(() => {
      this._data = null;
      this.setState({
        dataValid: false
      });
    })
  }
  errorListener = () => {
    this.setState({
      dataValid: false
    });
  };

  handleDraw = (ctx, frameCount) => {
    this.props.onDraw(this._data, ctx, frameCount);
  }
  render () {
    let statusMsg = null;
    if(!this.state.dataValid) {
      statusMsg = this.props.onReceivingError();
    }
    return (
      <CanvasWithStatus onDraw={ this.handleDraw }
        statusMsg={ statusMsg } statusVariant="danger"/>
    );
  }
}

export default CanvasDataFromRos;
