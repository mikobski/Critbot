import React from "react";
import RosTopic from "RosClient/Topic";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";

class StatusBattery extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 2000,
    topic: ROS_CONFIG.defaultTopics.statusBattery
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
      messageType: "sensor_msgs/BatteryState",
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
    if(message && message.cell_voltage && message.cell_voltage.length) {
      const voltage = parseFloat(message.cell_voltage[0]);
      this.setState({
        data: `${voltage.toFixed(1)} V`,
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
    let batVal = "--";
    let tdClass = "text-danger";
    if(this.state.valid) {
      tdClass = "text-success";
      batVal = this.state.data;
    }
    return (
      <tr> 
        <td>Battery:</td>
        <td className={ tdClass }>{ batVal }</td>
      </tr>
    );
  }
}

export default StatusBattery;