import React from "react";
import { Marker, Tooltip } from "react-leaflet";
import Polyline from 'react-leaflet-arrowheads'

class WaypointMarkers extends React.Component {
  render() {
    const waypoints = this.props.waypoints;
    const markersMap = waypoints.map((waypoint, index, array) => {
      let tooltipText = `Waypoint ${index+1}`;
      if(index === 0) {
        tooltipText = "Start waypoint";
      } else if(index === array.length-1) {
        tooltipText = "End waypoint";
      }
      return <Marker key={ index.toString() } position={waypoint.pos} draggable={ this.props.editable } onDrag={ (e) => {
        this.props.onMarkerDrag(e, index);
      }} onClick={ (e) => {
        this.props.onMarkerClick(e, index);
      }}>
          <Tooltip direction="bottom">{ tooltipText }</Tooltip>
        </Marker>
    });
    const linesMap = waypoints.map((waypoint, index, array) => {
      if(index === 0)
        return null;
      const options = {
        opacity: 0.7
      };
      return <Polyline key={ index.toString() } positions={[array[index-1].pos, waypoint.pos]} { ...options } arrowheads={{size: '20px'}}/>
    });

    return (
      <>
        { markersMap }
        { linesMap }
      </>
    );
  }
}

export default WaypointMarkers;