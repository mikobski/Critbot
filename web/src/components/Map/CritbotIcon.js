import ICON_IMAGE from "components/Map/critbot_icon.png";
import L from 'leaflet';
export const CritbotIcon = L.icon({
  iconUrl: ICON_IMAGE,
  iconSize: [35, 42],
  iconAnchor: [17, 21],
  popupAnchor: [17, 0],
});