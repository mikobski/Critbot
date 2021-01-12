import React from "react";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import "components/Map/Map.scss";
import "components/Map/fixLeaflet.js";
import { Button } from "react-bootstrap";
import { TrashFill } from "react-bootstrap-icons";

class Map extends React.Component {
  render() {
    return (
      <div className="Map-container">
        <div className="Map-header">
          <Button variant="secondary">
            <TrashFill/>
          </Button>
        </div>
        <MapContainer center={[51.505, -0.09]} zoom={13} scrollWheelZoom={true}>
          <TileLayer
            attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          <Marker position={[51.505, -0.09]}>
            <Popup>
              A pretty CSS3 popup. <br /> Easily customizable.
            </Popup>
          </Marker>
        </MapContainer>
      </div>
    )
  }
}

export default Map