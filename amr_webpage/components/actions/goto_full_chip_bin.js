'use strict'

const e = React.createElement

class ActionGoToTrashBin extends React.Component {
    constructor(props) {
        super(props)
        this.taskName = 'Chip bin no. 3'
        this.taskNumber = 1
    }

    onStartClick = (e) => {
        e.preventDefault()
        this.props.setTask(this.taskNumber)
        this.startTask()
    }
    onStopClick = (e) => {
        e.preventDefault()
        this.props.releaseTask()
        this.stopTask()
    }
    
    // circles
    ///startTask = () => {
    //    const msg = {
    //        linear: { x: 0.3 },
    //        angular: { z: 0.7 },
    //    }
     //   this.props.publishCmdVel(msg)
    //}

    // service
    startTask = () => {
        var request = new ROSLIB.ServiceRequest({
            label: 'trash_small_03',            
        });
        this.props.callingGotoClients(request)
        console.log("trash_small_03")
    }

    stopTask = () => {
        var request = new ROSLIB.ServiceRequest({
                        
        });
        this.props.callingGotoClients(request)
        console.log("randezvous with trash_small_03")
    }

    render() {
        return (
            <div className="control align-center">
                <label>{this.taskName}</label>
                <button
                    className="btn btn-info"
                    onClick={this.onStartClick}
                   /* disabled={this.props.taskRunning !== 0} */
                >go to full bin</button>

             
            </div>
        )
    }
}
