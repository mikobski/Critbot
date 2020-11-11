import React from "react";
import CanvasDataFromRos from "helpers/CanvasDataFromRos";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";

class Camera extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 500
  };
  _topic;

  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    const topicName = this.props.topic;
    this._topic = new RosTopic({
      ros: rosClient,
      name: topicName,
      messageType: "sensor_msgs/CompressedImage", 
      compression: "cbor"
    });
  }

  promiseNewData = (message) => {
    return new Promise((resolve, reject) => {
      const rawData = new Blob([message.data], { type: `image/rgb8; jpeg compressed bgr8` });
      createImageBitmap(rawData).then((imageBitmap) => {
        resolve(imageBitmap);
      }).catch((error) => {
        console.error(`[Critbot] Incorrect format of camera messeages on topic ${this.props.topic}`);
        reject();
      });
    });
  }

  handleDraw = (data, ctx, frameCount) => {
    if(data) {
      const imgDim = {
        height: data.height,
        width: data.width,
        ratio: data.width/data.height
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
      ctx.drawImage(data, dx, dy, width, height);
    }
  }

  handleReceivingError = () => {
    return  "No data received from camera";
  }
  
  render () {
    return (
      <CanvasDataFromRos topic={ this._topic } promiseNewData={ this.promiseNewData }
        onDraw={ this.handleDraw } 
        onReceivingError={ this.handleReceivingError }/>
    );
  }
}

export default Camera;
