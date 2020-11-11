import React from "react";
import { Table } from "react-bootstrap";
import "components/StatusDetailed/StatusDetailed.scss"

class StatusDetailed extends React.Component {
  render() {
    return (
      <div>
        <Table size="sm" className="StatusDetailed-table">
          <tbody>
            <tr>
              <td>Radio:</td>
              <td className="text-success">100%</td>
            </tr>
            <tr>
              <td>WiFi:</td>
              <td className="text-success">100%</td>
            </tr>
            <tr>
              <td>GPS:</td>
              <td className="text-danger">Error</td>
            </tr>
            <tr>
              <td>Battery:</td>
              <td className="text-success">100%</td>
            </tr>
            <tr>
              <td>Motors:</td>
              <td className="text-success">OK</td>
            </tr>
            <tr>
              <td>ROS:</td>
              <td className="text-success">OK</td>
            </tr>
          </tbody>
        </Table>
      </div>
    );
  }
}

export default StatusDetailed;