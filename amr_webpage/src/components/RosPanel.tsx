import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import RegularAction from "./actions/RegularAction";
import ActionManualControl from "./actions/ManualControl";
import OdometrySensor from "./sensors/OdometrySensor";
import LaserSensor from "./sensors/LaserSensor";

interface RosPanelProps {
  ros: ROSLIB.Ros;
}

export default function RosPanel({ ros }: RosPanelProps) {
  const [taskRunning, setTaskRunning] = useState(0);
  const [laserForward, setLaserForward] = useState(10);
  const [laserLeft, setLaserLeft] = useState(10);
  const [laserBackward, setLaserBackward] = useState(10);
  const [laserRight, setLaserRight] = useState(10);
  const [yaw, setYaw] = useState<number | null>(null);
  const [odometryLastMessage, setOdometryLastMessage] = useState({
    pose: { pose: { position: { x: 0, y: 0, z: 0 } } },
  });
  const pingInterval = useRef<NodeJS.Timeout>();

  const getYaw = useCallback((q: any) => {
    let angles = { roll: 0, pitch: 0, yaw: 0 };

    // roll (x-axis rotation)
    let sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    let cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    let sinp = 2 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1) {
      let signal = sinp > 0 ? 1 : -1;
      angles.pitch = Math.abs(Math.PI / 2) * signal;
    } else {
      angles.pitch = Math.asin(sinp);
    }

    // yaw (z-axis rotation)
    let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = Math.atan2(siny_cosp, cosy_cosp);

    return angles.yaw;
  }, []);
  const pingMsg = useCallback(() => {
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/any_topic",
      messageType: "std_msgs/String",
    });
    listener.subscribe(() => {
      listener.unsubscribe();
    });
  }, [ros]);
  const subscribeToSensors = useCallback(() => {
    // laser
    const laserSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: "/front_scan",
      messageType: "sensor_msgs/LaserScan",
    });
    laserSubscriber.subscribe((msg) => {
      const message = msg as { ranges: Array<number | null> };
      setLaserForward(
        message.ranges
          .slice(270, 540)
          .filter((r) => r != null)
          .reduce((acc, cur) => (cur < acc ? cur : acc), 10)
      );
      setLaserLeft(
        message.ranges
          .slice(630, 720)
          .filter((r) => r != null)
          .reduce((acc, cur) => (cur < acc ? cur : acc), 10)
      );
      setLaserBackward(
        message.ranges
          .slice(270, 225)
          .filter((r) => r != null)
          .reduce((acc, cur) => (cur < acc ? cur : acc), 10)
      );
      setLaserRight(
        message.ranges
          .slice(0, 90)
          .filter((r) => r != null)
          .reduce((acc, cur) => (cur < acc ? cur : acc), 10)
      );
    });

    // odom
    const odomSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: "/odom",
      messageType: "nav_msgs/Odometry",
    });
    odomSubscriber.subscribe((msg: any) => {
      let q = msg.pose.pose.orientation;
      let yaw = getYaw(q);
      setOdometryLastMessage(msg);
      setYaw(yaw);
    });
  }, []);

  // calling go_to_point service
  const callingGotoClients = useCallback((request: ROSLIB.ServiceRequest) => {
    var gotoClient = new ROSLIB.Service({
      ros,
      name: "/go_to_point",
      serviceType: "amr_localization/Poi_message",
    });

    gotoClient.callService(request, () => {});
  }, []);

  // calling Randez Vous service
  const callingRandezVousClients = useCallback(
    (request: ROSLIB.ServiceRequest) => {
      var rvClient = new ROSLIB.Service({
        ros,
        name: "/randezvous",
        serviceType: "std_srvs/Trigger",
      });

      rvClient.callService(request, () => {});
    },
    []
  );

  // calling elevator service
  const callingElevatorClients = useCallback(
    (request: ROSLIB.ServiceRequest) => {
      var elClient = new ROSLIB.Service({
        ros,
        name: "/elevator_service_server",
        serviceType: "amr_description/ElevatorServiceMessage",
      });

      elClient.callService(request, () => {});
    },
    []
  );

  // calling disengagement service
  const callingDisengagementClients = useCallback(
    (request: ROSLIB.ServiceRequest) => {
      var disClient = new ROSLIB.Service({
        ros,
        name: "/disengagement",
        serviceType: "std_srvs/Trigger",
      });

      disClient.callService(request, () => {});
    },
    []
  );

  const callingGazeboDeleteModelClients = useCallback(
    (request: ROSLIB.ServiceRequest) => {
      var gdmClient = new ROSLIB.Service({
        ros,
        name: "/gazebo/delete_model",
        serviceType: "gazebo_msgs/DeleteModel",
      });

      gdmClient.callService(request, () => {});
    },
    []
  );

  const publishCmdVel = useCallback((msg: any) => {
    // publish
    let topic = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    topic.publish(msg);
  }, []);

  // task manager
  const releaseTask = useCallback(() => {
    setTaskRunning(0);
  }, []);

  useEffect(() => {
    pingMsg();
    pingInterval.current = setInterval(pingMsg, 10 * 1000);
    subscribeToSensors();
  }, [pingMsg]);

  return (
    <div>
      {/* Robot actions */}
      <div id="robot-commands" className="column-40 align-center">
        <h2> AMR Pallet Truck actions</h2>
        <div className="column-50">
          <RegularAction
            handleRequest={callingGotoClients}
            setTask={setTaskRunning}
            requestData={{
              label: "trash_small_03",
            }}
            name="Go to full bin"
          />
          <RegularAction
            handleRequest={callingRandezVousClients}
            setTask={setTaskRunning}
            requestData={{}}
            name="Fork the bin"
          />

          <RegularAction
            handleRequest={callingElevatorClients}
            setTask={setTaskRunning}
            requestData={{
              elevator: "up",
            }}
            name="Elevator Up"
          />

          <RegularAction
            handleRequest={callingGotoClients}
            setTask={setTaskRunning}
            requestData={{
              label: "recycling_area",
            }}
            name="Go to recycling"
          />

          <RegularAction
            handleRequest={callingElevatorClients}
            setTask={setTaskRunning}
            requestData={{
              elevator: "down",
            }}
            name="Elevator down"
          />

          <RegularAction
            handleRequest={callingDisengagementClients}
            setTask={setTaskRunning}
            requestData={{}}
            name="Disengagement bin"
          />
          <RegularAction
            handleRequest={callingGazeboDeleteModelClients}
            setTask={setTaskRunning}
            requestData={{
              model_name: "europallet_01",
            }}
            name="Recycling bin"
          />
          <RegularAction
            handleRequest={callingGotoClients}
            setTask={setTaskRunning}
            requestData={{
              label: "load_area_bin_2",
            }}
            name="Go to empty bin"
          />
          <RegularAction
            handleRequest={callingGotoClients}
            setTask={setTaskRunning}
            requestData={{
              label: "download_area_bin_2",
            }}
            name="Go to cnc machine"
          />
          <RegularAction
            handleRequest={callingGotoClients}
            setTask={setTaskRunning}
            requestData={{
              label: "home",
            }}
            name="Go home"
          />
        </div>
        <div className="column-50">
          <ActionManualControl
            publishCmdVel={publishCmdVel}
            setTask={setTaskRunning}
            releaseTask={releaseTask}
          />
        </div>
      </div>

      {/* Robot sensors */}
      <div id="robot-sensors" className="column-30 align-center">
        <h2>Robot sensors</h2>
        <OdometrySensor lastMessage={odometryLastMessage} />
        <hr />
        <LaserSensor
          forward={laserForward}
          right={laserRight}
          left={laserLeft}
        />
      </div>
    </div>
  );
}
