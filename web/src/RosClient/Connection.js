import ROSLIB from "roslib";
import { EVENT_CONNECTED, EVENT_DISCONNECTED } from "./Constants.js";
import EventEmitter from "events";

class Connection extends EventEmitter {
	constructor(options) {
		super();

		let rosInstance;
		let connected = false;
		let connectScheduled = false;

		let onFail = () => {
			if(connected) {
				// Going from connected to disconnected, publish disconnected event
				this.emit(EVENT_DISCONNECTED);
			}
			connected = false;

			if (!connectScheduled) {	
				connectScheduled = true;
				setTimeout(this.connect, options.reconnectInterval);
			}
		};

		let onSuccess = () => {
			if(connected) {
				// Already in connected state...
				return;
			}
			connected = true;
			this.emit(EVENT_CONNECTED, rosInstance);
		};
		
		this.connect = () => {
			connectScheduled = false;
			if(!connected) {
				if(rosInstance) {
					rosInstance.close();
				}
				rosInstance = new ROSLIB.Ros({
					url: options.url
				});
				rosInstance.on("close", onFail);
				rosInstance.on("error", onFail);
				rosInstance.on("connection", onSuccess);
			}
		};	

		this.getUrl = () => {
			return options.url;
		};

		this.close = () => {
			rosInstance.close();
		};
	}
};

export default Connection;