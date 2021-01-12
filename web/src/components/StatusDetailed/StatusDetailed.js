import React from "react";
import { Table } from "react-bootstrap";
import RosTopic from "RosClient/Topic";
import "components/StatusDetailed/StatusDetailed.scss"
import StatusRow from "components/StatusDetailed/StatusRow"
import { RosContext } from "utils/RosContext";

class StatusDetailed extends React.Component {
  static contextType = RosContext;
  _topicBattery;
  _topicWifi;

  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    const topicName = this.props.topic;
    this._topicBattery = new RosTopic({
      ros: rosClient,
      name: this.props.topicBattery,
      messageType: "sensor_msgs/BatteryState"
    });
    this._topicWifi = new RosTopic({
      ros: rosClient,
      name: this.props.topicWifi,
      messageType: "std_msgs/String"
    });
  }
  promiseDataBattery = (message) => {
    return new Promise((resolve, reject) => {
      const voltage = parseFloat(message.cell_voltage[0]);
      resolve(`${voltage.toFixed(1)} V`);
    });
  }
  handleRenderBattery = (data, isError) => {
    const nameCol = <td>Battery:</td>
    if(isError) {
      return (
        <> 
          { nameCol }
          <td>--</td>
        </>
      )
    } else {
      return (
        <> 
          { nameCol }
          <td>{ data }</td>
        </>
      )
    }
  }

  promiseDataWifi = (message) => {
    return new Promise((resolve, reject) => {
      resolve(message.data);
    });
  }
  handleRenderWifi = (data, isError) => {
    const nameCol = <td>WiFi:</td>
    if(isError) {
      return (
        <> 
          { nameCol }
          <td>--</td>
        </>
      )
    } else {
      let wifiVal = data;
      if(wifiVal.length == 0) {
        wifiVal = "--";
      }
      return (
        <> 
          { nameCol }
          <td>{ wifiVal }</td>
        </>
      )
    }
  }

  render() {
    return (
      <div>
        <Table size="sm" className="StatusDetailed-table">
          <tbody>
          <StatusRow topic={ this._topicWifi }
              onNewData={ this.promiseDataWifi }
              onRender={ this.handleRenderWifi }/>
            <tr>
              <td>GPS:</td>
              <td className="text-danger">Error</td>
            </tr>
            <StatusRow topic={ this._topicBattery }
              onNewData={ this.promiseDataBattery }
              onRender={ this.handleRenderBattery }/>
          </tbody>
        </Table>
      </div>
    );
  }
}

export default StatusDetailed;