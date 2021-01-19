import ROSLIB from "roslib";

class RosClient extends ROSLIB.Ros {
  _reconnectTimeoutHandler = null;
  constructor(options) {
    super(options);
    options = options || {};
    this.Topic = undefined;
    this.Service = undefined;
    this.Param = undefined;
    this._reconnectTimeout = options.reconnectTimeout || 0;

    const handleConnectionFail = () => {
      if(this._reconnectTimeoutHandler == null) {
        const handleReconnect = () => {
          if(!this.isConnected) {
            this.connect(this.socket.url);
            this.emit("startReconnecting"); 
          }
          this._reconnectTimeoutHandler = null;
        };
        this._reconnectTimeoutHandler = setTimeout(handleReconnect, this._reconnectTimeout);
        this.emit("startReconnectingTimeout", this._reconnectTimeout); 
      }
    };

    if(this._reconnectTimeout) {
      this.on("close", handleConnectionFail);
      this.on("error", handleConnectionFail);
    }
  }

  connect(url) {
    if(!this.isConnected) {
      super.connect(url)
      this.emit("startConnection");  
    }
  }
}

export default RosClient;