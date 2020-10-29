import Connection from "./Connection";
import TopicManager from "./TopicManager";
import ServiceManager from "./ServiceManager";

const DEFAULT_OPTIONS = {
	url: "ws://localhost:9090",
	// Milliseconds before attempting another reconnect
	reconnectInterval: 5000
};

class Client {
	constructor(userOptions) {
		let options = Object.assign({}, DEFAULT_OPTIONS, userOptions);
		this.connection = new Connection(options);
		this.service = new ServiceManager(this.connection);
		this.topic = new TopicManager(this.connection);
	}
};

export default Client;