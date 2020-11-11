import React from "react";
import RosClient from "RosClient";

export const RosContext = React.createContext(new RosClient());
RosContext.displayName = "RosContext";
