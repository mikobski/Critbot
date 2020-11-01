import React from "react";

class Measurements extends React.Component {
  render() {
    const containerStyle = {
      display: "flex",
      height: "100%",
      justifyContent: "center",
      alignItems: "center",
      background: "rgba(86, 150, 124, 0.05)"
    }
    return (
      <div style={ containerStyle }>
        <div>Measurements</div>
      </div>
    )
  }
}

export default Measurements