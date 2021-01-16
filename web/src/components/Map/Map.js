import React, { createRef } from "react";
import { Map as MapLeaf, TileLayer, Marker, Tooltip, Polyline } from "react-leaflet";

import "leaflet/dist/leaflet.css";
import "components/Map/Map.scss";
import "components/Map/fixLeaflet.js";
import WaypointMarkers from "components/Map/WaypointMarkers";
import { CritbotIcon } from "components/Map/CritbotIcon";
import RotatedMarker from "components/Map/RotatedMarker";
import { Button, ButtonGroup } from "react-bootstrap";
import { PlayFill, PauseFill } from "react-bootstrap-icons";
import RosTopic from "RosClient/Topic";
import ActionClient from "RosClient/ActionClient";
import Goal from "RosClient/Goal";
import { RosContext } from "utils/RosContext";
import { ROS_CONFIG } from "utils/RosConfig";
import { Mode } from "utils/Mode";

class Map extends React.Component {
  static contextType = RosContext;
  static defaultProps = {
    noDataTimeout: 2000,
    odomTopic: ROS_CONFIG.defaultTopics.mapOdom
	};
  _odomTopic;
  _actionClient;
  _mapRef;
  constructor(props, context) {
    super(props, context);
    const rosClient = this.context;
    this.state = {
      geoHome: [54.37137110207873, 18.613119721412662, 0],
      odom: [0, 0, 0],
      odomAvailable: false,
      waypoints: [],
      isDriving: false
    }
    this._odomTopic = new RosTopic({
      ros: rosClient,
      name: this.props.odomTopic,
      messageType: "nav_msgs/Odometry",
      timeout: this.props.noDataTimeout
    });
    this._actionClient = new ActionClient({
      ros: rosClient,
      serverName: ROS_CONFIG.defaultActionServers.mapWaypoints.server,
      actionName: ROS_CONFIG.defaultActionServers.mapWaypoints.action,
      omitStatus: false,
      omitFeedback: true,
      omitResult: false
    });
    this._mapRef = createRef();
  }
  componentDidMount() {
    this._odomTopic.subscribe(this.odomListener, this.odomErrorListener);
    this._actionClient.cancel();
  }
  componentDidUpdate(prevProps) {
    if(this.props.mode !== Mode.AUTO && prevProps.mode === Mode.AUTO) {
      console.log("cancel");
      this._actionClient.cancel();
    }
  }
  componentWillUnmount() {
    this._odomTopic.unsubscribe(this.odomListener, this.odomErrorListener);
  }
  odomListener = (message) => {
    if(message && message.pose) {
      this.setState({
        odomAvailable: true,
        odom: [message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.orientation.z],
      });
    } else {
      this.setState({odomAvailable: false});
    }
  };
  odomErrorListener = () => {
    this.setState({odomAvailable: false});
  };
  _geoToOdom(geo, origin) {    
    const origAng = origin[2]*Math.PI/180;
    let odom = [0, 0, 0];
    if(this._mapRef.current) {
      const magicScale = 5/2.909385959752482720;
      const leaflet = this._mapRef.current.leafletElement;
      const projection = leaflet.options.crs.projection;
      const geoProj = projection.project({lat: geo[0], lng: geo[1]});
      const origProj = projection.project({lat: origin[0], lng: origin[1]});
      const proj = {
        x: geoProj.x - origProj.x, 
        y: geoProj.y - origProj.y
      };   
      odom[0] = (Math.cos(-origAng)*proj.x - Math.sin(-origAng)*proj.y)/magicScale;
      odom[1] = (Math.cos(-origAng)*proj.y + Math.sin(-origAng)*proj.x)/magicScale;
    }
    odom[2] = (geo[2] - origin[2])/180;
    return odom;
  }
  _odomToGeo(odom, origin) {    
    const origAng = origin[2]*Math.PI/180;
    const odomRotated = [
      Math.cos(origAng)*odom[0] - Math.sin(origAng)*odom[1],
      Math.cos(origAng)*odom[1] + Math.sin(origAng)*odom[0]
    ];
    let geo = [origin[0], origin[1], origin[2]];
    if(this._mapRef.current) {
      const magicScale = 5/2.909385959752482720;
      const leaflet = this._mapRef.current.leafletElement;
      const projection = leaflet.options.crs.projection;
      const geoProj = projection.project({lat: geo[0], lng: geo[1]});
      const proj = {
        x: geoProj.x + odomRotated[0]*magicScale, 
        y: geoProj.y + odomRotated[1]*magicScale
      };
      let geoUnproj = projection.unproject(proj);      
      geo[0] = geoUnproj.lat;
      geo[1] = geoUnproj.lng;
    }
    geo[2] += odom[2]*180;
    return geo;
  }
  handleStart = () => {
    this.setState(() => {
      console.log("start gola");
      const waypoint = this.state.waypoints[0];
      const pos = this._geoToOdom([...waypoint.pos, 0], this.state.geoHome);
      let goal = new Goal({
        actionClient: this._actionClient,
        goalMessage: {
          target_pose: {
            header: {
              frame_id: "odom",
              stamp: 0
            },
            pose: {
              position: {
                x: pos[0],
                y: pos[1],
                z: 0
              },
              orientation: {
                x: 0, y:0, z:0, w:1
              }
            }
          }
        }
      });
      goal.on('result', function(result) {
        console.log(this);
        console.log(result);
      });
      goal.on('status', (status) => {
        //console.log(status);
      });
      goal.send();
      return { isDriving: true }
    });
  };
  handlePause = () => {
    this.setState(() => {
      this._actionClient.cancel();
      return { isDriving: false }
    });
  };
  handleMapClick = (e) => {
    if(this.state.isDriving) return;
    this.setState((prevState) => {   
      const wp = prevState.waypoints;
      let newWp = wp.concat({
        pos: [e.latlng.lat, e.latlng.lng]
      });
      return  {waypoints: newWp}; 
    });  
  };
  handleMarkerClick = (e, i) => {
    if(this.state.isDriving) return;
    this.setState((prevState) => {   
      let wp = [...prevState.waypoints];
      wp.splice(i, 1);
      return  {waypoints: wp}; 
    });  
  };
  handleMarkerDrag = (e, i) => {
    if(this.state.isDriving) return;
    this.setState((prevState) => {
      const wp = prevState.waypoints;
      wp[i].pos = [e.latlng.lat, e.latlng.lng]; 
      return {waypoints: wp};
    });    
  };
  render() { 
    const robotGeo = this._odomToGeo(this.state.odom, this.state.geoHome);
    return (
      <div className="Map-container">
        <div className="Map-header" style={{ paddingRight: "0"}}>
          <div>
            <Button variant="primary" onClick={ () => {} }>
              Follow robot
            </Button>
          </div>
          { this.props.mode == Mode.AUTO &&
            <ButtonGroup>
              <Button variant="primary" disabled={ this.state.isDriving } onClick={ this.handleStart }>
                <PlayFill/> Start mission
              </Button>
              <Button variant="primary" disabled={ !this.state.isDriving } onClick={ this.handlePause }>
                <PauseFill/> Pause mission
              </Button>
            </ButtonGroup>
          }
        </div>
        <MapLeaf ref={ this._mapRef }center={this.state.geoHome.slice(0, 2)} zoom={19} scrollWheelZoom={true} onClick={ this.handleMapClick }>
          <TileLayer
            attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            maxZoom={21}
            maxNativeZoom={19}
          />
          <WaypointMarkers waypoints={ this.state.waypoints } editable={ !this.state.isDriving }
            onMarkerDrag={ this.handleMarkerDrag } onMarkerClick={ this.handleMarkerClick }/>
            { this.state.odomAvailable &&
              <RotatedMarker icon={ CritbotIcon } position={ robotGeo.slice(0, 2) } rotationAngle={ robotGeo[2] }/>
            }
        </MapLeaf>
      </div>
    )
  }
}

export default Map