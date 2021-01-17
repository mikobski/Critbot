import React from "react";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";
import Plot from 'react-plotly.js';

class Measurements extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 2000,
    topic: ROS_CONFIG.defaultTopics.measPlot
	};
  _topic;
  _data;

  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    this.state = {
      valid: false,
      dataRevision: 0
    }
    this._topic = new RosTopic({
      ros: rosClient,
      name: this.props.topic,
      messageType: "nav_msgs/Odometry",
      throttle_rate: 200,
      timeout: this.props.noDataTimeout
    });
    this._data = [{
      x: [],
      y: []
    }, {
      x: [],
      y: []
    }];
  }
  componentDidMount() {
    this._topic.subscribe(this.topicListener, this.errorListener);
  }
  componentWillUnmount() {
    this._topic.unsubscribe(this.topicListener, this.errorListener);
  }
  topicListener = (message) => {
    if(message && message.header) {
      const time = new Date(message.header.stamp.secs*1000 + Math.round(message.header.stamp.nsecs/1000/1000));
      this._data[0].y.push(message.twist.twist.linear.x);
      this._data[0].x.push(time);
      this._data[1].y.push(message.twist.twist.angular.z);
      this._data[1].x.push(time);
      const xRange = 10*1000;
      let firstInRange = 0;
      const lastTime = this._data[0].x[this._data[0].x.length - 1].getTime();
      for(const i in this._data[0].x) {
        if(this._data[0].x[i].getTime() < lastTime - xRange) {
          firstInRange = i;
        }
      }
      this._data[0].y = this._data[0].y.slice(firstInRange);
      this._data[0].x = this._data[0].x.slice(firstInRange);
      this._data[1].y = this._data[1].y.slice(firstInRange);
      this._data[1].x = this._data[1].x.slice(firstInRange);
      this.setState((prevState) => {
        return {
          valid: true,
          dataRevision: prevState.dataRevision + 1
        }
      });
    } else {
      this.setState({
        valid: false
      });
    }
  };
  errorListener = () => {
    this.setState({valid: false});
  };

  render() {
    const containerStyle = {
      display: "flex",
      height: "100%",
      justifyContent: "center",
      alignItems: "center",
      background: "rgba(86, 150, 124, 0.05)"
    };
    let firstTimestamp = Date.now();
    if(this._data[0].x.length > 0) {
      firstTimestamp = this._data[0].x[0].getTime();
    }
    let layout = {
      width: 955,
      height: 400,
      margin: {
        l: 60,
        r: 20,
        t: 20,
        b: 65
      },
      legend: {
        x: 0.05,
        y: 0.05
      },
      xaxis: {
        title: "Time",
        tickmode: "linear",
        tick0: new Date(Math.ceil(firstTimestamp/1000)*1000),
        dtick: 1000
      },
      yaxis: {
        title: "Velocity",
        autorange: false,
        range: [-5, 5]
      },
      datarevision: this.state.dataRevision
    };
    let data = [
      {
        x: this._data[0].x,
        y: this._data[0].y,
        name: "Linear [m/s]",
        type: 'scatter',
        mode: 'lines+points',
        marker: { color: 'red' },
      },
      {
        x: this._data[1].x,
        y: this._data[1].y,
        name: "Angular [rad/s]",
        type: 'scatter',
        mode: 'lines+points',
        marker: { color: 'blue' },
      }
    ];
    return (
      <div style={ containerStyle }>
          <Plot
          data={ data }
          layout={ layout }
          debug={ true }
          revision={ this.state.dataRevision }
          config={{
            scrollZoom: true
          }}
        />
      </div>
    )
  }
}

export default Measurements