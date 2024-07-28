import { useEffect } from "react";

interface MapProps {
  ros: ROSLIB.Ros;
}

export default function Map({ ros }: MapProps) {
  useEffect(() => {
    try {
      const ros2dViewer = new window.ROS2D.Viewer({
        divID: "map",
        width: 750,
        height: 550,
      });
      window.NAV2D.OccupancyGridClientNav({
        ros: ros,
        rootObject: ros2dViewer.scene,
        viewer: ros2dViewer,
        serverName: "/move_base",
        topic: "/map",
      });
    } catch (error) {
      console.log("Nav error");
      console.log(error);
    }
  }, [ros]);

  return (
    <div id="map" className="dark align-center">
      This is the map
    </div>
  );
}
