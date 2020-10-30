import "./VerticalRow.scss";

function VerticalRow(props) {
    const stylesRow = {flexBasis: props.basis};
    return (
        <div className="vertical-row" style={ stylesRow }>
            { props.children }
        </div>
    )
}

export default VerticalRow
