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
      const gridClient = new window.ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: ros2dViewer.scene,
      });

      gridClient.on("change", () => {
        ros2dViewer.scaleToDimensions(
          gridClient.currentGrid.width,
          gridClient.currentGrid.height
        );
      });
    } catch (error) {
      console.log("Nav error");
      console.log(error);
    }
  }, [ros]);

  return <div id="map" className="align-center"></div>;
}
