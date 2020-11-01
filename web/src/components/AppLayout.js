import { Navbar } from "react-bootstrap";
import NavbarStatus from "components/NavbarStatus/NavbarStatus";
import NavbarModeSelector from "components/NavbarModeSelector";
import PanelsLayout from "components/PanelsLayout";
import VerticalContainer from "helpers/VerticalContainer";
import VerticalCol from "helpers/VerticalRow";

const AppLayout = (props) => {
  return (
    <VerticalContainer>
      <VerticalCol basis="auto">
        <Navbar bg="dark" variant="dark">
          <Navbar.Brand>Critbot</Navbar.Brand>
          <NavbarStatus/>
          <NavbarModeSelector/>
        </Navbar>
      </VerticalCol>
      <VerticalCol basis="100%">
        <PanelsLayout/>
      </VerticalCol>
    </VerticalContainer>
  );
}

export default AppLayout;