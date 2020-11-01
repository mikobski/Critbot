import React from "react";

class Map extends React.Component {
  render() {
    const containerStyle = {
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      background: "rgba(86,61,124,.15)"
    }
    return (
      <div style={ containerStyle }>
        <div>Map</div>
      </div>
    )
  }
}

export default Map