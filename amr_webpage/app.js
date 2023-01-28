'use strict';

const e = React.createElement;

let ros

class AppComponent extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
            connected: false,
        }
        this.defaultRosbridgeAddress = 'wss://i-00221d5d24e3c2c98.robotigniteacademy.com/5bb64df8-92fe-4dad-bc04-b076367e63f3/rosbridge/'
    }

    onConnectBtn = (e) => {
        e.preventDefault()
        try {
            ros = new ROSLIB.Ros({
                url: document.getElementById('rosbridge_address').value
            })
        } catch (ex) {
            console.log(ex)
            //
        }
        ros.on('connection', () => {
            console.log('Connected to websocket server.')
            this.setState({
                connected: true
            })
            this.setState({ connected: true })
        })
        ros.on('error', (error) => {
            console.log('Error connecting to websocket server: ', error)
        })
        ros.on('close', () => {
            console.log('Connection to websocket server closed.')
            this.setState({ connected: false })
            this.setState({
                connected: false
            })
        })
    }

    onDisconnectBtn = (e) => {
        e.preventDefault()
        ros.close()
    }

    render() {
        return (
            <div id="container">
                <Header />

                <div id="content">
                    {/* Menu */}
                    <div id="menu" className="column-30">
                        <div className="control">
                            <label>Rosbridge address</label>
                            <input
                                defaultValue={this.defaultRosbridgeAddress}
                                type="text"
                                disabled={this.state.connected}
                                id="rosbridge_address"
                            />
                        </div>
                        {!this.state.connected &&
                            <div className="control align-right">
                                <button className="btn btn-success" type="button" onClick={this.onConnectBtn}>Connect</button>
                            </div>
                        }
                        {this.state.connected &&
                            <div className="control align-right">
                                <button className="btn btn-danger" type="button" onClick={this.onDisconnectBtn}>Disconnect</button>
                            </div>
                        }
                    </div>

                    {this.state.connected && <RosPanel ros={ros} />}

                    {/* Clear */}
                    <div className="clear"></div>
                </div>

                <Footer />
            </div>
        )
    }
}

const appContainer = document.querySelector('#app')
ReactDOM.render(e(AppComponent), appContainer)
