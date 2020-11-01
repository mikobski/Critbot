import React from "react";
import RosClient from "RosClient/RosLibJsClient";

export const RosContext = React.createContext(new RosClient({
  url: "ws://localhost:9090"
}));
RosContext.displayName = "RosContext";
