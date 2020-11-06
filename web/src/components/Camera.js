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
      console.log({scale: scale, width: width, height: height});
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
