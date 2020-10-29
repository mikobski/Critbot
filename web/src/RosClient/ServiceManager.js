import ROSLIB from "roslib";

class ServiceManager {
	constructor(connection) {
		this.call = (name, serviceType, payload) => {
			return connection.getInstance().then(function(ros) {
				return new Promise(function(resolve) {
					var service = new ROSLIB.Service({
						ros: ros,
						name: name,
						serviceType: serviceType
					});
					var request = new ROSLIB.ServiceRequest(payload);
					service.callService(request, function(response) {
						resolve(response);
					});
				});
			});
		};
	}
};

export default ServiceManager;
