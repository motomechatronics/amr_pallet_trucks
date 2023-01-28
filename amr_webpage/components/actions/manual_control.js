'use strict'

const e = React.createElement

class ActionManualControl extends React.Component {
    constructor(props) {
        super(props)
        this.taskName = 'AMR Manual Control'
        this.taskNumber = 2
        this.taskInterval = null
    }


    onMoveForwardClick = (e) => {
        e.preventDefault()
        this.props.setTask(this.taskNumber)
        this.MoveForwardTask()
    }

    onMoveBackwardClick = (e) => {
        e.preventDefault()
        this.props.setTask(this.taskNumber)
        this.MoveBackwardTask()
    }

    onMoveTurnCWClick = (e) => {
        e.preventDefault()
        this.props.setTask(this.taskNumber)
        this.MoveTurnCWTask()
    }

    onMoveTurnCCWClick = (e) => {
        e.preventDefault()
        this.props.setTask(this.taskNumber)
        this.MoveTurnCCWTask()
    }

    onStopClick = (e) => {
        e.preventDefault()
        this.props.releaseTask()
        this.stopTask()
    }


    MoveForwardTask = () => {
        this.taskInterval = setInterval(() => {
            let msg         
                
                msg = { linear: { x: 0.1 }, angular: { z: 0.0 } }
        
            this.props.publishCmdVel(msg)
        }, 50)
    }

    MoveBackwardTask = () => {
        this.taskInterval = setInterval(() => {
            let msg         
                
                msg = { linear: { x: -0.1 }, angular: { z: 0.0 } }
        
            this.props.publishCmdVel(msg)
        }, 50)
    }

    MoveTurnCWTask = () => {
        this.taskInterval = setInterval(() => {
            let msg         
                
                msg = { linear: { x: 0.0 }, angular: { z: -0.1 } }
        
            this.props.publishCmdVel(msg)
        }, 50)
    }

    MoveTurnCCWTask = () => {
        this.taskInterval = setInterval(() => {
            let msg         
                
                msg = { linear: { x: 0.0 }, angular: { z: 0.1 } }
        
            this.props.publishCmdVel(msg)
        }, 50)
    }

    stopTask = () => {
        const msg = {
            linear: { x: 0 },
            angular: { z: 0 },
        }
        this.props.publishCmdVel(msg)
        clearInterval(this.taskInterval)
    }

    render() {
        return (
            <div className="control align-center">
                <label>{this.taskName}</label>
                <button
                    className="btn btn-success"
                    onClick={this.onMoveForwardClick} 
                    //disabled={this.props.taskRunning !== 0}
                >move forward</button>
                <br />
                <p> </p>
                <button
                    className="btn btn-success"
                    onClick={this.onMoveBackwardClick} 
                    //disabled={this.props.taskRunning !== 0}
                >move backward</button>
                <br />
                <p> </p>
                <button
                    className="btn btn-success"
                    onClick={this.onMoveTurnCWClick} 
                    //disabled={this.props.taskRunning !== 0}
                >move turn CW</button>
                <br />
                <p> </p>
                <button
                    className="btn btn-success"
                    onClick={this.onMoveTurnCCWClick} 
                    //disabled={this.props.taskRunning !== 0}
                >move turn CCW</button>
                <br />
                <p> </p>
                <button
                    className="btn btn-danger"
                    onClick={this.onStopClick}
                    //disabled={this.props.taskRunning !== this.taskNumber}
                >stop</button>

            </div>
        )
    }
}
