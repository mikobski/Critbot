import { Navbar } from "react-bootstrap";
import NavbarStatus from "components/NavbarStatus/NavbarStatus";
import NavbarModeSelector from "components/NavbarModeSelector";
import PanelsLayout from "components/PanelsLayout/PanelsLayout";
import { Mode } from "utils/Mode";

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
        <PanelsLayout mode={ Mode.MANUAL }/>
      </div>
    </div>
  );
}

export default AppLayout;