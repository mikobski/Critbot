import ROSLIB from "roslib";

class RosClient extends ROSLIB.Ros {
  constructor(options) {
    super(options);
    this.Topic = undefined;
    this.Service = undefined;
    this.Param = undefined;
  }
}

export default RosClient;