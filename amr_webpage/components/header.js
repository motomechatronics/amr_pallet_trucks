'use strict';

const e = React.createElement;

class Header extends React.Component {
    constructor(props) {
        super(props)
    }

    render() {
        return (
            <div id="header" className="dark align-center">
                <h2>Chip Bins Manager Control Panel v.1.0</h2>
            </div>
        )
    }
}