import React from "react";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";

class StatusGps extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 2000,
    topic: ROS_CONFIG.defaultTopics.statusGps
	};
  _topic;

  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    this.state = {
      valid: false,
      data: null
    }
    this._topic = new RosTopic({
      ros: rosClient,
      name: this.props.topic,
      messageType: "sensor_msgs/NavSatFix",
      throttle_rate: 500,
      timeout: this.props.noDataTimeout
    });
  }
  componentDidMount() {
    this._topic.subscribe(this.topicListener, this.errorListener);
  }
  componentWillUnmount() {
    this._topic.unsubscribe(this.topicListener, this.errorListener);
  }
  topicListener = (message) => {
    if(message && message.status) {
      this.setState({
        data: message.status.status,
        valid: true
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
    let gpsVal = "--";
    let tdClass = "text-danger";
    if(this.state.valid) {
      if(this.state.data >= 0) {
        tdClass = "text-success";
        gpsVal = "Fixed";
      } else if(this.state.data < 0) {
        tdClass = "text-warning";
        gpsVal = "Not fixed";
      }
    }
    return (
      <tr> 
        <td>GPS:</td>
        <td className={ tdClass }>{ gpsVal }</td>
      </tr>
    );
  }
}

export default StatusGps;