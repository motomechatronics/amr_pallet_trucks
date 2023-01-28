'use strict'

const e = React.createElement

class RosPanel extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
            taskRunning: 0,
            // laser data
            laserForward: 10,
            laserLeft: 10,
            laserBackward: 10,
            laserRight: 10,
            // odom data
            odometryLastMessage: { pose: { pose: { position: { x: 0, y: 0, z: 0 } } } },
            // orientation
            yaw: null,
        }
        this.pingInterval = null
        this.count = 0
    }

    componentDidMount = () => {
        this.pingMsg()
        this.pingInterval = setInterval(this.pingMsg, 10 * 1000)
        this.subscribeToSensors()
    }

    pingMsg = () => {
        var listener = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/any_topic',
            messageType: 'std_msgs/String'
        })
        listener.subscribe()
        // unsubscribe after 1 second
        setTimeout(() => { listener.unsubscribe() }, 1 * 1000)
    }

    componentWillUnmount = () => {
        clearInterval(this.pingInterval)
    }

    // publish
    publishCmdVel = (msg) => {
        // publish
        let topic = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        })
        topic.publish(msg)
    }

    getYaw = (q) => {
        let angles = { roll: 0, pitch: 0, yaw: 0 }

        // roll (x-axis rotation)
        let sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        let cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        angles.roll = Math.atan2(sinr_cosp, cosr_cosp)

        // pitch (y-axis rotation)
        let sinp = 2 * (q.w * q.y - q.z * q.x)
        if (Math.abs(sinp) >= 1) {
            let signal = sinp > 0 ? 1 : -1
            angles.pitch = Math.abs(M_PI / 2) * signal
        }
        else {
            angles.pitch = Math.asin(sinp)
        }

        // yaw (z-axis rotation)
        let siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        angles.yaw = Math.atan2(siny_cosp, cosy_cosp)

        return angles.yaw
    }

    // subscribe
    subscribeToSensors = () => {
        // laser
        var laserSubscriber = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/front_scan',
            messageType: 'sensor_msgs/LaserScan'
        })
        laserSubscriber.subscribe((msg) => {
            this.setState({
                laserForward: msg.ranges.slice(270, 540).filter(r => r != null).reduce((acc, cur) => cur < acc ? cur : acc, 10),
                laserLeft: msg.ranges.slice(630, 720).filter(r => r != null).reduce((acc, cur) => cur < acc ? cur : acc, 10),
                laserBackward: msg.ranges.slice(270, 225).filter(r => r != null).reduce((acc, cur) => cur < acc ? cur : acc, 10),
                laserRight: msg.ranges.slice(0, 90).filter(r => r != null).reduce((acc, cur) => cur < acc ? cur : acc, 10),
            })
        })

        // odom
        var odomSubscriber = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/odom',
            messageType: 'nav_msgs/Odometry'
        })
        odomSubscriber.subscribe((msg) => {
            let q = msg.pose.pose.orientation
            let yaw = this.getYaw(q)
            this.setState({
                odometryLastMessage: msg,
                yaw,
            })
        })
    }

    // calling go_to_point service
    callingGotoClients = (request) => {
        var gotoClient = new ROSLIB.Service({
            ros: this.props.ros,
            name : '/go_to_point',
            serviceType : 'amr_localization/Poi_message'
        });
        
        gotoClient.callService(request, function(result) {
        
        });
    }
    
    // calling Randez Vous service
    callingRandezVousClients = (request) => {
        var rvClient = new ROSLIB.Service({
            ros: this.props.ros,
            name : '/randezvous',
            serviceType : 'std_srvs/Trigger'
        });
        
        rvClient.callService(request, function(result) {
        
        });
    }
    
    // calling elevator service
    callingElevatorClients = (request) => {
        var elClient = new ROSLIB.Service({
            ros: this.props.ros,
            name : '/elevator_service_server',
            serviceType : 'amr_description/ElevatorServiceMessage'
        });
        
        elClient.callService(request, function(result) {
        
        });
    }

     // calling disengagement service
    callingDisengagementClients = (request) => {
        var disClient = new ROSLIB.Service({
            ros: this.props.ros,
            name : '/disengagement',
            serviceType : 'std_srvs/Trigger'
        });
        
        disClient.callService(request, function(result) {
        
        });
    }

    callingGazeboDeleteModelClients = (request) => {
        var gdmClient = new ROSLIB.Service({
            ros: this.props.ros,
            name : '/gazebo/delete_model',
            serviceType : 'gazebo_msgs/DeleteModel'
        });
        
        gdmClient.callService(request, function(result) {
        
        });
    }
    


    // task manager
    setTask = (task) => {
        this.setState({
            taskRunning: task
        })
    }
    releaseTask = () => {
        this.setState({
            taskRunning: 0
        })
    }

    render() {
        return (
            <React.Fragment>
                {/* Robot actions */}
                <div id="robot-commands" className="column-40 align-center">
                    <h2> AMR Pallet Truck actions</h2>
                    <div className="column-50">
                        <ActionGoToTrashBin
                            taskRunning={this.state.taskRunning}
                            callingGotoClients={this.callingGotoClients}                            
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />
                        <ActionRandezVousTrashBin                            
                            taskRunning={this.state.taskRunning}                            
                            callingRandezVousClients={this.callingRandezVousClients}
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />

                        <ActionElevatorUpTrashBin                           
                            taskRunning={this.state.taskRunning}                            
                            callingElevatorClients={this.callingElevatorClients}
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />

                        <ActionMoveChipsBinToRecicling
                            taskRunning={this.state.taskRunning}
                            callingGotoClients={this.callingGotoClients}                           
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />

                        <ActionElevatorDownChipsBin                             
                            taskRunning={this.state.taskRunning}                            
                            callingElevatorClients={this.callingElevatorClients}
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        /> 

                        <ActionDisengagementChipBin                       
                            taskRunning={this.state.taskRunning}                            
                            callingDisengagementClients={this.callingDisengagementClients}
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />
                        <ActionRecyclingBin
                            taskRunning={this.state.taskRunning}
                            callingGazeboDeleteModelClients={this.callingGazeboDeleteModelClients}                           
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />                        
                        <ActionGoToEmptyBin
                            taskRunning={this.state.taskRunning}
                            callingGotoClients={this.callingGotoClients}                           
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />
                        <ActionGoToCnCMachine
                            taskRunning={this.state.taskRunning}
                            callingGotoClients={this.callingGotoClients}                           
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />
                        <ActionGoHome
                            taskRunning={this.state.taskRunning}
                            callingGotoClients={this.callingGotoClients}                           
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}
                        />
                    </div>
                    <div className="column-50">
                        <ActionManualControl
                            forward={this.state.laserForward}
                            taskRunning={this.state.taskRunning}
                            publishCmdVel={this.publishCmdVel}
                            setTask={this.setTask}
                            releaseTask={this.releaseTask}                        
                        />

                    </div>
                </div>

                {/* Robot sensors */}
                <div id="robot-sensors" className="column-30 align-center">
                    <h2>Robot sensors</h2>
                    <SensorOdometry
                        lastMessage={this.state.odometryLastMessage}
                    />
                    <hr />
                    <SensorLaser
                        forward={this.state.laserForward}
                        right={this.state.laserRight}
                        backward={this.state.laserBackward}
                        left={this.state.laserLeft}
                    />
                </div>



            </React.Fragment>
        )
    }
}
