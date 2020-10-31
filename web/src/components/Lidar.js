import React from "react";
import Canvas from "../helpers/Canvas";

class Lidar extends React.Component {
  _topic;
  _data = [];

  componentDidMount() {
    const topicManager = this.props.ros.topic;
    const topicName = this.props.topic;
    this._topic = topicManager.subscribe(topicName, "sensor_msgs/LaserScan", this.topicListener);
  }
  componentWillUnmount() {
    this._topic.dispose();
  }

  topicListener = (message) => {
    if(message.header.frame_id === "laser") {
      this._data = [];
      let angle = message.angle_min;
      for(const range of message.ranges) {
        this._data.push([
          Math.cos(angle+Math.PI/2)*range,
          Math.sin(angle+Math.PI/2)*range
        ]);
        angle += message.angle_increment;
      }
    }
  }

  calcScale(ctx) {
    const { width, height } = ctx.canvas;
    const { physicalWidth, physicalHeight } = this.props;
    let scale = 1;
    if(physicalWidth === 0 || physicalHeight === 0) {
      return scale;
    }
    return Math.min(width/physicalWidth, height/physicalHeight);
  }

  drawGrid(ctx, scale, center) {
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

  drawPoints(ctx, scale, center) {
    const circleDim = 2;
    const color = "rgba(0, 120, 255, 0.8)";
    ctx.fillStyle = color;
    for(let i = 0; i < this._data.length; i++) {
      const point = this._data[i];
      ctx.beginPath()
      ctx.arc(center.x + point[0]*scale, center.y + point[1]*scale, circleDim, 0, 2*Math.PI)
      ctx.fill()
    }
  }

  draw = (ctx, frameCount) => {
    const scale = this.calcScale(ctx);
    const center = {
      x: ctx.canvas.width/2,
      y: ctx.canvas.height/2
    }

    this.drawGrid(ctx, scale, center);
    this.drawPoints(ctx, scale, center);
  }

  render () {
    return (
      <div style={{ border: "1px solid red"}}>
        <Canvas draw={ this.draw }/>
      </div>
    );
  }
}
Lidar.defaultProps = {
  physicalWidth: 12,
  physicalHeight: 12,
  gridMax: 10,
  gridCount: 4
}

export default Lidar;
