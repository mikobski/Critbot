import React from "react";
import { Badge } from "react-bootstrap";
import { RosContext } from "utils/RosContext";

const ConectionStatus = {
    CONNECTED: "CONNECTED",
    DISCONNECTED: "DISCONNECTED"
};

class StatusConnection extends React.Component {
    static contextType = RosContext;
    constructor(props) {
      super(props)
      this.state = {
          connectionStatus: ConectionStatus.DISCONNECTED
      };
    }
    componentDidMount() {
        this.context.connection.on("connected", this.handleConnected);
        this.context.connection.on("disconnected", this.handleDisconnected);
    }
    componentWillUnmount() {
        this.context.connection.removeListener("connected", this.handleConnected);
        this.context.connection.removeListener("disconnected", this.handleDisconnected);
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
                <i>({ this.context.connection.getUrl() })</i>
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