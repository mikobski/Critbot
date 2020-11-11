import React from "react";
import CanvasWithStatus from "helpers/CanvasWithStatus";
import { RosContext } from "utils/RosContext";

class CanvasDataFromRos extends React.PureComponent {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 500
  };
  _noDataTimeoutHandler = null;
  _topic;
  _data = null;

  constructor(props) {
    super(props);
    console.log(this.props);
    this.state = {
      isReceiving: false
    };
  }

  componentDidMount() {
    const topic = this.props.topic;
    topic.subscribe(this.topicListener);
    topic.ros.on("close", this.onReceivingError);
    topic.ros.on("error", this.onReceivingError);
  }
  componentWillUnmount() {
    const topic = this.props.topic;
    topic.unsubscribe(this.topicListener);
    topic.ros.off("close", this.onReceivingError);
    topic.ros.off("error", this.onReceivingError);
    if(this._noDataTimeoutHandler != null) {
      clearTimeout(this._noDataTimeoutHandler);
    }
  }

  onReceivingError = () => {
    this.setState({
      isReceiving: false
    });
  };

  topicListener = (message) => {
    this.props.promiseNewData(message).then((data) => {
      this._data = data;
    }).catch(() => {
      this._data = null;
      this.onReceivingError();
    })
    this.setState({
      isReceiving: true
    });
    if(this._noDataTimeoutHandler != null) {
      clearTimeout(this._noDataTimeoutHandler);
    }
    this._noDataTimeoutHandler = setTimeout(this.onReceivingError, this.props.noDataTimeout);
  }

  draw = (ctx, frameCount) => {
    this.props.onDraw(this._data, ctx, frameCount);
  }

  render () {
    let statusMsg = null;
    if(!this.state.isReceiving) {
      statusMsg = this.props.onReceivingError();
    }
    return (
      <CanvasWithStatus draw={ this.draw }
        statusMsg={ statusMsg } statusVariant="danger"/>
    );
  }
}

export default CanvasDataFromRos;
