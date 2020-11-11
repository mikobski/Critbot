import React from "react";
import Canvas from "helpers/Canvas";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";

class Camera extends React.Component {
  static contextType = RosContext;
  _topic;
  _data = null;

  componentDidMount() {
    const rosClient = this.context;
    const topicName = this.props.topic;
    this._topic = new RosTopic({
      ros: rosClient,
      name: topicName,
      messageType: "sensor_msgs/CompressedImage", 
      compression: "cbor"
    });
    this._topic.subscribe(this.topicListener);
  }
  componentWillUnmount() {
    this._topic.unsubscribe(this.topicListener);
  }

  topicListener = (message) => {
    const rawData = new Blob([message.data], { type: `image/rgb8; jpeg compressed bgr8` });
    createImageBitmap(rawData).then((imageBitmap) => {
      this._data = imageBitmap;
    }).catch((error) => {
      console.error(`[Critbot] Incorrect format of camera messeages on topic ${this.props.topic}`);
    });
  }

  draw = (ctx, frameCount) => {
    if(this._data) {
      const imgDim = {
        height: this._data.height,
        width: this._data.width,
        ratio: this._data.width/this._data.height
      }
      const canvasDim = {
        height: ctx.canvas.height,
        width: ctx.canvas.width,
        ratio: ctx.canvas.width/ctx.canvas.height
      }

      let scale = 1;
      if(imgDim.ratio < canvasDim.ratio) {
        scale = canvasDim.height/imgDim.height;
      } else {
        scale = canvasDim.width/imgDim.width;        
      }

      const width = imgDim.width*scale;
      const height = imgDim.height*scale;
      const dx = (canvasDim.width - width)/2;
      const dy = (canvasDim.height - height)/2
      ctx.drawImage(this._data, dx, dy, width, height);
    }
  }

  render () {
    return (
      <Canvas draw={ this.draw }/>
    );
  }
}

export default Camera;
