import React from "react";
import { Map as MapLeaf, TileLayer, Marker, Tooltip, Polyline } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import "components/Map/Map.scss";
import "components/Map/fixLeaflet.js";
import { Button, ButtonGroup } from "react-bootstrap";
import { PlayFill, PauseFill, TrashFill, PlusCircle } from "react-bootstrap-icons";

class Map extends React.Component {
  constructor(props, context) {
    super(props, context);
    this.state = {
      geoHome: [51.505, -0.09],
      waypoints: [
        {
          pos: [51.5051, -0.091]
        },
        {
          pos: [51.505, -0.09]
        },
        {
          pos: [51.5052, -0.092]
        }
      ]
    }
  }
  handleMapClick = (e) => {
    console.log(e);
  };
  handleMarkerClick = (e, i) => {
    console.log([i, e]);
  };
  render() {
    const markersMap = this.state.waypoints.map((waypoint, index, array) => {
      let tooltipText = `Waypoint ${index+1}`;
      if(index === 0) {
        tooltipText = "Start waypoint";
      } else if(index === array.length-1) {
        tooltipText = "End waypoint";
      }
      return <Marker key={ index.toString() } position={waypoint.pos} onClick={ (e) => {
        this.handleMarkerClick(e, index);
      }}>
          <Tooltip direction="bottom">{ tooltipText }</Tooltip>
        </Marker>
    });
    const linesMap = this.state.waypoints.map((waypoint, index, array) => {
      if(index === 0)
        return null;
      const options = {
        opacity: 0.5
      };
      return <Polyline key={ index.toString() } positions={[waypoint.pos, array[index-1].pos]} pathOptions={ options }/>
    });
    return (
      <div className="Map-container">
        <div className="Map-header" style={{ paddingRight: "0"}}>
          <ButtonGroup className="Map-mission-btns">
            <Button variant="primary">
              <PlayFill/> Start mission
            </Button>
            <Button variant="primary">
              <PauseFill/> Pause mission
            </Button>
          </ButtonGroup>
          <ButtonGroup className="Map-edit-btns">
            <Button variant="primary">
              <PlusCircle/> Add marker
            </Button>
            <Button variant="primary">
              <TrashFill/> Remove marker
            </Button>
          </ButtonGroup>
        </div>
        <MapLeaf center={this.state.geoHome} zoom={13} scrollWheelZoom={true} onClick={ this.handleMapClick }>
          <TileLayer
            attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          { markersMap }
          { linesMap }
        </MapLeaf>
      </div>
    )
  }
}

export default Map