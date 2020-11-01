import RosClient from "RosClient/RosLibJsClient";
import React from "react";

export const RosContext = React.createContext(new RosClient({
  url: "ws://localhost:9090"
}));
RosContext.displayName = "RosContext";
