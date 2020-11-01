import React from "react";
import Canvas from "helpers/Canvas";
import { RosContext } from "utils/RosContext";

class Camera extends React.Component {
  static contextType = RosContext;
  _topic;
  _data = null;

  componentDidMount() {
    const topicManager = this.context.topic;
    const topicName = this.props.topic;
    this._topic = topicManager.subscribe(topicName, "sensor_msgs/CompressedImage", 
      this.topicListener, { compresion: "cbor" });
  }
  componentWillUnmount() {
    this._topic.dispose();
  }

  topicListener = (message) => {
    const rawData = new Blob([message.data], { type: `image/rgb8; jpeg compressed bgr8` });
    createImageBitmap(rawData).then((imageBitmap) => {
      this._data = imageBitmap;
    });
  }

  draw = (ctx, frameCount) => {
    if(this._data) {
      ctx.drawImage(this._data, 0, 0);
    }
  }

  render () {
    return (
      <Canvas draw={ this.draw }/>
    );
  }
}

export default Camera;
