import React from "react";

class Map extends React.Component {
  render() {
    const containerStyle = {
      display: "flex",
      height: "100%",
      justifyContent: "center",
      alignItems: "center",
      background: "rgba(150, 61, 124, 0.05)"
    }
    return (
      <div style={ containerStyle }>
        <div>Map</div>
      </div>
    )
  }
}

export default Map