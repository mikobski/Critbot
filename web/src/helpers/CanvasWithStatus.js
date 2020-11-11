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
      top: "50%",
      left: "0",
      transform: "translate(0, -50%)",
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