import React from "react";
import CanvasDataFromRos from "helpers/CanvasDataFromRos";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";

class Lidar extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    physicalWidth: 12,
    physicalHeight: 12,
    gridMax: 10,
    gridCount: 4,
    noDataTimeout: 1000,
    topic: ROS_CONFIG.defaultTopics.lidar
  };
  _topic;

  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    const topicName = this.props.topic;
    this._topic = new RosTopic({
      ros: rosClient,
      name: topicName,
      messageType: "sensor_msgs/LaserScan", 
      compression: "cbor",
      timeout: this.props.noDataTimeout
    });
  }

  promiseNewData = (message) => {
    return new Promise((resolve, reject) => {
      if(message.header.frame_id === "laser") {
        let data = [];
        let angle = message.angle_min;
        for(const range of message.ranges) {
          data.push([
            Math.cos(angle+Math.PI/2)*range,
            Math.sin(angle+Math.PI/2)*range
          ]);
          angle += message.angle_increment;
        }
        resolve(data);
      }
    });
  }

  _calcScale(ctx) {
    const { width, height } = ctx.canvas;
    const { physicalWidth, physicalHeight } = this.props;
    let scale = 1;
    if(physicalWidth === 0 || physicalHeight === 0) {
      return scale;
    }
    return Math.min(width/physicalWidth, height/physicalHeight);
  }

  _drawGrid(ctx, scale, center) {
    const lineWidth = 1;
    const lineColour = "#6c757d";
    const crosshairSize = 10;
    const fontSize = 16;

    ctx.strokeStyle = lineColour;

    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(center.x-crosshairSize/2, center.y);
    ctx.lineTo(center.x+crosshairSize/2, center.y);
    ctx.moveTo(center.x, center.y-crosshairSize/2);
    ctx.lineTo(center.x, center.y+crosshairSize/2);
    ctx.stroke();

    ctx.fillStyle = lineColour;
    ctx.font = `${fontSize}px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue"`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const gridStep = this.props.gridMax/this.props.gridCount;
    for(let i = 0; i < this.props.gridCount; i++) {
      const radius = gridStep*(1 + i);
      ctx.beginPath();
      const clearAngle = Math.asin(fontSize/2 / (radius*scale) * 1.2);
      ctx.arc(center.x, center.y, radius*scale, clearAngle, 2*Math.PI-clearAngle);
      ctx.stroke();

      ctx.fillText(radius, center.x+radius*scale, center.y);
    }
  }

  _drawPoints(data, ctx, scale, center) {
    const circleDim = 2;
    const color = "rgba(0, 120, 255, 0.8)";
    ctx.fillStyle = color;
    for(let i = 0; i < data.length; i++) {
      const point = data[i];
      ctx.beginPath()
      ctx.arc(center.x + point[0]*scale, center.y + point[1]*scale, circleDim, 0, 2*Math.PI)
      ctx.fill()
    }
  }

  handleDraw = (data,ctx, frameCount) => {
    const scale = this._calcScale(ctx);
    const center = {
      x: ctx.canvas.width/2,
      y: ctx.canvas.height/2
    }

    this._drawGrid(ctx, scale, center);
    if(data !== null) {
      this._drawPoints(data, ctx, scale, center);
    }
  }
  
  handleReceivingError = () => {
    return  "No data received from LIDAR";
  }

  render () {
    return (
      <CanvasDataFromRos topic={ this._topic } promiseNewData={ this.promiseNewData }
        onDraw={ this.handleDraw } 
        onReceivingError={ this.handleReceivingError }/>
    );
  }
}

export default Lidar;
