import { Navbar } from "react-bootstrap";
import NavbarStatus from "components/NavbarStatus/NavbarStatus";
import NavbarModeSelector from "components/NavbarModeSelector";
import PanelsLayout from "components/PanelsLayout/PanelsLayout";
import { MODE } from "utils/Mode";

const AppLayout = (props) => {
const StylesContainer = {
  height: "100%",
  display: "flex",
  flexDirection: "column"
}

  return (
    <div style={ StylesContainer }>
      <div style={{ flexBasis: "auto" }}>
        <Navbar bg="dark" variant="dark">
          <Navbar.Brand>Critbot</Navbar.Brand>
          <NavbarStatus/>
          <NavbarModeSelector/>
        </Navbar>
      </div>
      <div style={{ flexBasis: "100%", overflow: "hidden" }}>
        <PanelsLayout mode={ MODE.manual }/>
      </div>
    </div>
  );
}

export default AppLayout;