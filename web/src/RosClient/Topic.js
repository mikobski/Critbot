import ROSLIB from "roslib";

class Topic extends ROSLIB.Topic {
  _timeoutHandler = null;
  constructor(options) {
    super(options);
    this.ros.on("close", this._onError);
    this.ros.on("topic-error", this._onError);
    if(options && options.timeout) {
      this.timeout = options.timeout;
      this.on("message", this._handleMessage);
      this._timeoutHandler = setTimeout(this._onTimeout, this.timeout);
    }
  }
  subscribe(callback, errorCallback) {
    super.subscribe(callback);
    if(typeof errorCallback === "function") {
      this.on("topic-error", errorCallback);
      this.on("timeout", errorCallback);
    }
  }
  unsubscribe(callback, errorCallback) {
    super.unsubscribe(callback);
    if(typeof errorCallback === "function") {
      this.off("topic-error", errorCallback);
      this.off("timeout", errorCallback);
      if(this.listeners("timeout").length) {
        return;
      }
    }
    if(this._timeoutHandler !== null) {
      clearTimeout(this._timeoutHandler);
    }
  }

  _onError = () => {
    this.emit("topic-error");
  }
  _onTimeout = () => {
    this.emit("timeout");
  }
  _handleMessage = () => {
    if(this._timeoutHandler !== null) {
      clearTimeout(this._timeoutHandler);
    }
    this._timeoutHandler = setTimeout(this._onTimeout, this.timeout);
  };
}

export default Topic;