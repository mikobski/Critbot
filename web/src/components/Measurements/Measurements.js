import React from "react";

class Measurements extends React.Component {
  render() {
    const containerStyle = {
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      background: "rgba(86,61,124,.15)"
    }
    return (
      <div style={ containerStyle }>
        <div>Measurements</div>
      </div>
    )
  }
}

export default Measurements