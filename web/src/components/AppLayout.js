function AppLayout(props) {
    const containerStyle = {
        height: "100%",
        display: "flex",
        flexDirection: "column"
    }
    const contentStyle = {
        flexBasis: "100%"
    }

    return (
        <div style={containerStyle}>
            <div>
                {props.nav}
            </div>
            <div style={contentStyle}>
                {props.content}
            </div>
        </div>
    )
}

export default AppLayout