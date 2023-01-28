'use strict'

const e = React.createElement

class SensorLaser extends React.Component {
    constructor(props) {
        super(props)
    }

    render() {
        return (
            <div className="control">
                <label>Obstacles</label>
                <table>
                    <tbody>
                        <tr>
                            <td></td>
                            <td>Forward: <span>{this.props.forward.toFixed(2)}</span></td>
                            <td></td>
                        </tr>
                        <tr>
                            <td colSpan="3">&nbsp;</td>
                        </tr>
                        <tr>
                            <td>Left: <span>{this.props.left.toFixed(2)}</span></td>
                            <td> TT </td>
                            <td>Right: <span>{this.props.right.toFixed(2)}</span></td>
                        </tr>
                        <tr>
                            <td colSpan="3">&nbsp;</td>
                        </tr>
                    
                    </tbody>
                </table>
            </div>
        )
    }
}
