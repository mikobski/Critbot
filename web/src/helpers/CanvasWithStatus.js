import React from "react"
import { Alert } from "react-bootstrap";
import Canvas from "helpers/Canvas";

const CanvasWithStatus = props => {
  const { statusVariant, statusMsg, ...propsRest } = props;
  let statusAlert = null
  if(statusMsg) {
    const stylesContainer = {
      width: "100%",
      position: "absolute",
      top: "0",
      left: "0",
      padding: "1rem"
    }
    statusAlert = <div style={ stylesContainer }>
      <Alert variant={statusVariant}>{statusMsg}</Alert>
    </div>
  }
  return (
    <>
      <Canvas {...propsRest}/>
      { statusAlert }
    </>
  );
}

export default CanvasWithStatus;