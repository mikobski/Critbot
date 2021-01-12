import React from "react";
import { Table } from "react-bootstrap";
import StatusWifi from "components/StatusDetailed/StatusWifi";
import StatusBattery from "components/StatusDetailed/StatusBattery";
import "components/StatusDetailed/StatusDetailed.scss"

class StatusDetailed extends React.Component {
  render() {
    return (
      <div>
        <Table size="sm" className="StatusDetailed-table">
          <tbody>
            <StatusWifi topic={ this.props.topicWifi }/>
            <tr>
              <td>GPS:</td>
              <td className="text-danger">Error</td>
            </tr>
            <StatusBattery topic={ this.props.topicBattery }/>
          </tbody>
        </Table>
      </div>
    );
  }
}

export default StatusDetailed;