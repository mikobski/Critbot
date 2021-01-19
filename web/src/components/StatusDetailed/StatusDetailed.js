import React from "react";
import { Table } from "react-bootstrap";
import StatusWifi from "components/StatusDetailed/StatusWifi";
import StatusBattery from "components/StatusDetailed/StatusBattery";
import StatusGps from "components/StatusDetailed/StatusGps";
import "components/StatusDetailed/StatusDetailed.scss"

class StatusDetailed extends React.Component {
  render() {
    return (
      <div>
        <Table size="sm" className="StatusDetailed-table">
          <tbody>
            <StatusWifi/>
            <StatusGps/>
            <StatusBattery/>
          </tbody>
        </Table>
      </div>
    );
  }
}

export default StatusDetailed;