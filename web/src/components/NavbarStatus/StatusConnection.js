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
        const rosClient = this.context;
        rosClient.on("connection", this.handleConnected);
        rosClient.on("close", this.handleDisconnected);
        rosClient.on("error", this.handleDisconnected);
    }
    componentWillUnmount() {
        const rosClient = this.context;
        rosClient.removeListener("close", this.handleDisconnected);
        rosClient.removeListener("connection", this.handleConnected);
        rosClient.removeListener("error", this.handleDisconnected);
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
        const rosClient = this.context;
        let statusElement;
        if(this.state.connectionStatus === ConectionStatus.CONNECTED) {
            statusElement = <Badge variant="success">
                OK &nbsp;
                <i>({ rosClient.socket.url })</i>
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