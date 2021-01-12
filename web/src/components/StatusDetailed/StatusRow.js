import React from "react";
import { RosContext } from "utils/RosContext";

class StatusRow extends React.Component {
	static contextType = RosContext;
	static defaultProps = {
		noDataTimeout: 2000
	};
	_noDataTimeoutHandler = null;
	_topic;

	constructor(props) {
		super(props);
		this.state = {
			data: null,
			isError: true
		};
	}

	onError = () => {
		this.setState({
			isError: true
		});
	}
  componentDidMount() {
		const topic = this.props.topic;
		console.log(topic);
    topic.subscribe(this.topicListener);
    topic.ros.on("close", this.onError);
    topic.ros.on("error", this.onError);
  }
  componentWillUnmount() {
    const topic = this.props.topic;
    topic.unsubscribe(this.topicListener);
    topic.ros.off("close", this.onError);
    topic.ros.off("error", this.onError);
    if(this._noDataTimeoutHandler != null) {
      clearTimeout(this._noDataTimeoutHandler);
    }
	}

  topicListener = (message) => {
		console.log(message);
    this.props.onNewData(message).then((data) => {
			console.log(data);
			this.setState({
				data: data,
				isError: false
			});
    }).catch(() => {
			this.setState({
				data: null,
				isError: true
			});
    })
    if(this._noDataTimeoutHandler != null) {
      clearTimeout(this._noDataTimeoutHandler);
    }
    this._noDataTimeoutHandler = setTimeout(this.onError, this.props.noDataTimeout);
  }
	
	render() {
		return (
			<tr>
				{ this.props.onRender(this.state.data, this.state.isError) }
			</tr>
		)
	}
}

export default StatusRow;
