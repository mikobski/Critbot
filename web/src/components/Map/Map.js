import React, { createRef } from "react";
import { Map as MapLeaf, TileLayer, Marker, Tooltip } from "react-leaflet";
import Polyline from 'react-leaflet-arrowheads'

import "leaflet/dist/leaflet.css";
import "components/Map/Map.scss";
import "components/Map/fixLeaflet.js";
import WaypointMarkers from "components/Map/WaypointMarkers";
import { CritbotIcon } from "components/Map/CritbotIcon";
import RotatedMarker from "components/Map/RotatedMarker";
import { Button, ButtonGroup } from "react-bootstrap";
import { PlayFill, XCircle as XIcon } from "react-bootstrap-icons";
import RosTopic from "RosClient/Topic";
import RosService from "RosClient/Service";
import ActionClient from "RosClient/ActionClient";
import Goal from "RosClient/Goal";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";
import { Mode } from "utils/Mode";
import qte from "quaternion-to-euler";

class Map extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 2000,
    odomTopic: ROS_CONFIG.defaultTopics.mapOdom,
    navSatTopic: ROS_CONFIG.defaultTopics.mapNavSat,
    setWaypointsService: ROS_CONFIG.defaultTopics.mapSetWaypoints,
    cancelMissionService: ROS_CONFIG.defaultTopics.mapCancelMission,
    missionStatusTopic: ROS_CONFIG.defaultTopics.mapMissionStatus
	};
  _odomTopic;
  _navSatTopic;
  _setWpService;
  _cancelMissionService;
  _missionStateTopic;
  _mapRef;
  _mapCenter;
  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    this.state = {
      geo: {
        lat: 54.37137110207873, 
        lng: 18.613119721412662,
        yaw: 0,
        posAvailable: false,
        yawAvailable: false
      },
      waypoints: [],
      startWaypoint: null,
      inMission: false
    }
    this._odomTopic = new RosTopic({
      ros: rosClient,
      name: this.props.odomTopic,
      messageType: "nav_msgs/Odometry",
      timeout: this.props.noDataTimeout
    });
    this._navSatTopic = new RosTopic({
      ros: rosClient,
      name: this.props.navSatTopic,
      messageType: "sensor_msgs/NavSatFix",
      timeout: this.props.noDataTimeout
    });
    this._setWpService = new RosService({
      ros: rosClient,
      name: this.props.setWaypointsService,
      type: "waypoint_navigation/SetWaypoints"
    });
    this._cancelMissionService = new RosService({
      ros: rosClient,
      name: this.props.cancelMissionService,
      type: "waypoint_navigation/CancelMission"
    });
    this._missionStatusTopic = new RosTopic({
      ros: rosClient,
      name: this.props.missionStatusTopic,
      messageType: "std_msgs/String",
      timeout: this.props.noDataTimeout
    });
    this._mapRef = createRef();
    this._mapCenter = null;
  }
  componentDidMount() {
    this._odomTopic.subscribe(this.odomListener, this.odomErrorListener);
    this._navSatTopic.subscribe(this.navSatListener, this.navSatErrorListener);
    this._missionStatusTopic.subscribe(this.missionStatusListener, this.missionStatusError);
  }
  componentDidUpdate(prevProps) {
    if(this.props.mode !== Mode.AUTO && prevProps.mode === Mode.AUTO) {
      this.handleCancel();
    }
  }
  componentWillUnmount() {
    this._odomTopic.unsubscribe(this.odomListener, this.odomErrorListener);
    this._navSatTopic.unsubscribe(this.navSatListener, this.navSatErrorListener);
    this._missionStatusTopic.unsubscribe(this.missionStatusListener, this.missionStatusError);
  }
  odomListener = (message) => {
    if(message && message.pose) {
      this.setState((prevState) => {
        let geo = Object.assign({}, prevState.geo);
        const qa = message.pose.pose.orientation;
        const euler = qte([qa.x, qa.y, qa.z, qa.w]);
        geo.yaw = -euler[0]*180/Math.PI;
        geo.yawAvailable = true;
        return {geo: geo};
      });
    } else {
      this.setState((prevState) => {
        let geo = Object.assign({}, prevState.geo);
        geo.yawAvailable = false;
        return {geo: geo};
      });
    }
  };
  odomErrorListener = () => {
    this.setState((prevState) => {
      let geo = Object.assign({}, prevState.geo);
      geo.yawAvailable = false;
      return {geo: geo};
    });
  };
  navSatListener = (message) => {
    if(message) {
      this.setState((prevState) => {
        let geo = Object.assign({}, prevState.geo);
        geo.lat = message.latitude;
        geo.lng = message.longitude;
        geo.posAvailable = true;
        return {geo: geo};
      });
    } else {
      this.setState((prevState) => {
        let geo = Object.assign({}, prevState.geo);
        geo.posAvailable = false;
        return {geo: geo};
      });
    }
  };
  navSatErrorListener = () => {
    this.setState((prevState) => {
      let geo = Object.assign({}, prevState.geo);
      geo.posAvailable = false;
      return {geo: geo};
    });
  };
  missionStatusListener = (msg) => {
    if(msg.data === "IN_MISSION") {
      this.setState({ inMission: true });
    } else if(msg.data === "IDLE") {
      this.setState({ inMission: false });
    } else {
      console.error(`Unknown mission status! ${msg}`);  
    }
  }
  missionStatusError = () => {
  };
  handleStart = () => {
    let waypoints = this.state.waypoints.map((wp) => {
      return {
        x: wp.lat,
        y: wp.lng,
        theta: 0
      }
    });
    this._setWpService.callService({
      waypoints: waypoints
    }, (msg) => {
      if(msg.statusCode === 0) {
        this.setState( { 
          inMission: true,
          startWaypoint: {
            lat: this.state.geo.lat,
            lng: this.state.geo.lng
          } });
      } else {
        console.error(`Setting waypoint failed! ${msg.statusText}`);
      }
    }, (msg) => {
      console.error(`Setting waypoint failed! ${msg}`);
      this.setState( { inMission: false });
    });
  };
  handleCancel = () => {
    this._cancelMissionService.callService({}, (msg) => {
      if(msg.statusCode === 0) {
        this.setState( { inMission: false });
      } else {
        console.error(`Setting waypoint failed! ${msg.statusText}`);
      }
    }, (msg) => {
      console.error(`Mission cancel failed! ${msg}`);
    });    
  };
  handleMapClick = (e) => {
    if(this.state.inMission) return;
    this.setState((prevState) => {   
      const wp = prevState.waypoints;
      let newWp = wp.concat({
        lat: e.latlng.lat, 
        lng: e.latlng.lng
      });
      return  {waypoints: newWp}; 
    });  
  };
  handleMarkerClick = (e, i) => {
    if(this.state.inMission) return;
    this.setState((prevState) => {   
      let wp = [...prevState.waypoints];
      wp.splice(i, 1);
      return  {waypoints: wp}; 
    });  
  };
  handleMarkerDrag = (e, i) => {
    if(this.state.inMission) return;
    this.setState((prevState) => {
      const wp = prevState.waypoints;
      wp[i].lat = e.latlng.lat
      wp[i].lng = e.latlng.lng; 
      return {waypoints: wp};
    });    
  };
  render() { 
    let mapCenter = [0, 0];
    if(this._mapCenter == null) {
      if(this.state.geo.posAvailable) {
        this._mapCenter = [this.state.geo.lat, this.state.geo.lng];
        mapCenter = this._mapCenter;
      }
    } else {
      mapCenter = this._mapCenter;
    }
    const curPosLineOptions = {
      opacity: 0.3,
      color: "#6c757d",
      dashArray: [4]
    };
    return (
      <div className="Map-container">
        <div className="Map-header">
          { this.props.mode == Mode.AUTO &&
            <ButtonGroup>
              <Button variant="primary" disabled={ this.state.inMission } onClick={ this.handleStart }>
                <PlayFill/> Start mission
              </Button>
              <Button variant="primary" disabled={ !this.state.inMission } onClick={ this.handleCancel }>
                <XIcon/> Cancel mission
              </Button>
            </ButtonGroup>
          }
        </div>
        <MapLeaf ref={ this._mapRef } center={ mapCenter } zoom={ 18 } scrollWheelZoom={true} onClick={ this.handleMapClick }
          zoomAnimation={ false } markerZoomAnimation={ false }>
          <TileLayer
            attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            maxZoom={21}
            maxNativeZoom={19}
          />
          <WaypointMarkers waypoints={ this.state.waypoints } editable={ !this.state.inMission }
            onMarkerDrag={ this.handleMarkerDrag } onMarkerClick={ this.handleMarkerClick }/>
            { this.state.geo.yawAvailable && this.state.geo.posAvailable &&
              <RotatedMarker icon={ CritbotIcon } position={ this.state.geo } rotationAngle={ this.state.geo.yaw }/>              
            }
            { this.state.geo.yawAvailable && this.state.geo.posAvailable && 
              this.state.waypoints.length > 0 &&
              <Polyline positions={[
                (this.state.inMission ? [this.state.startWaypoint.lat, this.state.startWaypoint.lng] : [this.state.geo.lat, this.state.geo.lng]), 
                [this.state.waypoints[0].lat, this.state.waypoints[0].lng]
              ]} { ...curPosLineOptions } arrowheads={{size: '20px'}}/>
            }
        </MapLeaf>
      </div>
    )
  }
}

export default Map