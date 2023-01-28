'use strict'

const e = React.createElement

class SensorOdometry extends React.Component {
    constructor(props) {
        super(props)
    }

    componentDidMount() {
        console.log(this.props.lastMessage)
    }

    render() {
        return (
            <div className="control">
                <label>Robot position</label>
                <div>
                    <span>X: <u>{this.props.lastMessage.pose.pose.position.x.toFixed(2)}</u></span>
                    <br />
                    <span>Y: <u>{this.props.lastMessage.pose.pose.position.y.toFixed(2)}</u></span>
                </div>
            </div>
        )
    }
}
