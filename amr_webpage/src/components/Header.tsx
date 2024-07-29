import { useState } from "react";
import { Route } from "../App";

const DEFAULT_ROSBRIDGE_ADDRESS =
  "wss://i-00221d5d24e3c2c98.robotigniteacademy.com/5bb64df8-92fe-4dad-bc04-b076367e63f3/rosbridge/";

interface HeaderProps {
  onConnect: (rosbridgeAddress: string) => void;
  onDisconnect: () => void;
  connected: boolean;
  routes: Route[];
  onRouteChange: (routeId: Route["id"]) => void;
}

export default function Header({
  onConnect,
  onDisconnect,
  onRouteChange,
  connected,
  routes,
}: HeaderProps) {
  const [rosbridgeAddress, setRosbridgeAddress] = useState(
    DEFAULT_ROSBRIDGE_ADDRESS
  );

  return (
    <div id="header" className="dark align-center">
      <div className="nav-bar">
        <h1>J16 MOTo</h1>
        <div className="nav-items">
          {routes.map((route) => (
            <button
              onClick={() => onRouteChange(route.id)}
              className="nav-item"
            >
              {route.name}
            </button>
          ))}
        </div>
      </div>
      <h2>Chip Bins Manager Control Panel v.1.0</h2>
      <div className="rosbridge-address">
        <input
          className="control"
          placeholder="Rosbridge Address"
          value={rosbridgeAddress}
          onChange={(e) => {
            setRosbridgeAddress(e.target.value);
          }}
          type="text"
          disabled={connected}
        />
        {!connected && (
          <button
            className="btn btn-success"
            type="button"
            onClick={() => {
              onConnect(rosbridgeAddress);
            }}
          >
            Connect
          </button>
        )}
        {connected && (
          <button
            className="btn btn-danger"
            type="button"
            onClick={onDisconnect}
          >
            Disconnect
          </button>
        )}
      </div>
    </div>
  );
}
