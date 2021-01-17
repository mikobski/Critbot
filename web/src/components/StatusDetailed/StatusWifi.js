import React from "react";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";

class StatusWifi extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
		noDataTimeout: 2000,
    topic: ROS_CONFIG.defaultTopics.statusWifi
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
      messageType: "std_msgs/String",
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
    if(message && message.data && message.data.length) {
      this.setState({
        data: message.data,
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
    let wifiVal = "--";
    let tdClass = "text-danger";
    if(this.state.valid) {
      tdClass = "text-success";
      wifiVal = this.state.data;
    }
    return (
      <tr> 
        <td>WiFi:</td>
        <td className={ tdClass }>{ wifiVal }</td>
      </tr>
    );
  }
}

export default StatusWifi;