import React from "react";
import { Badge } from "react-bootstrap";

const ConectionStatus = {
    CONNECTED: "CONNECTED",
    DISCONNECTED: "DISCONNECTED"
};

class StatusConnection extends React.Component {
    constructor(props) {
      super(props)
      this.state = {
          connectionStatus: ConectionStatus.DISCONNECTED
      };
    }
    componentDidMount() {
        this.props.ros.connection.on("connected", this.handleConnected);
        this.props.ros.connection.on("disconnected", this.handleDisconnected);
    }
    componentWillUnmount() {
        this.props.ros.connection.removeListener("connected", this.handleConnected);
        this.props.ros.connection.removeListener("disconnected", this.handleDisconnected);
    }

    handleConnected = () => {
        this.setState({
            connectionStatus: ConectionStatus.CONNECTED
        });
    };
    handleDisconnected = () => {
        this.setState({
            connectionStatus: ConectionStatus.DISCONNECTED
        });
    };

    render() {
        let statusElement;
        if(this.state.connectionStatus === ConectionStatus.CONNECTED) {
            statusElement = <Badge variant="success">
                OK &nbsp;
                <i>({ this.props.ros.connection.getUrl() })</i>
            </Badge> ;
        } else if(this.state.connectionStatus === ConectionStatus.DISCONNECTED) {
            statusElement = <Badge variant="danger">Disconnected</Badge>;
        } else {
            statusElement = "";
        }
        return (
            <>
                <span>Connection: </span>
                { statusElement }                
            </>
        );
    }
}

export default StatusConnection;