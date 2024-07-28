import { ReactNode, useCallback, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useSnackbar } from "notistack";
import Header from "./components/Header";
import Footer from "./components/Footer";
import RosPanel from "./components/RosPanel";
import Map from "./components/map/Map";

export interface Route {
  id: "controls" | "map";
  name: string;
  component: ReactNode;
}

function App() {
  const [connected, setConnected] = useState(false);
  const [route, setRoute] = useState<Route["id"]>("controls");
  const ros = useRef<ROSLIB.Ros>();
  const { enqueueSnackbar } = useSnackbar();

  const ROUTES: Record<Route["id"], Route> = useMemo(
    () => ({
      controls: {
        component: ros.current ? <RosPanel ros={ros.current} /> : null,
        name: "Controls",
        id: "controls",
      },
      map: {
        component: ros.current ? <Map ros={ros.current} /> : null,
        name: "Map",
        id: "map",
      },
    }),
    [ros.current]
  );

  const onConnect = useCallback(
    (rosbridgeAddress: string) => {
      try {
        ros.current = new ROSLIB.Ros({
          url: rosbridgeAddress,
        });
      } catch (error) {
        return;
      }

      ros.current.on("connection", () => {
        enqueueSnackbar("Successfully connected to websocket server.");
        setConnected(true);
      });
      ros.current.on("error", () => {
        enqueueSnackbar("Error occurred connecting to websocket server", {
          variant: "error",
        });
      });
      ros.current.on("close", () => {
        if (connected) {
          enqueueSnackbar("Connection to websocket server closed.", {
            variant: "info",
          });
        }
        setConnected(false);
      });
    },
    [enqueueSnackbar]
  );

  return (
    <div id="container">
      <Header
        connected={connected}
        onConnect={onConnect}
        onDisconnect={() => {
          ros.current?.close();
        }}
        routes={Object.values(ROUTES)}
        onRouteChange={setRoute}
      />

      <div id="content">
        {connected && ros.current && ROUTES[route].component}

        {/* Clear */}
        <div className="clear"></div>
      </div>

      <Footer />
    </div>
  );
}

export default App;
