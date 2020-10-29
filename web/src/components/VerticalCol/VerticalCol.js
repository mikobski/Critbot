import './VerticalCol.scss';

function VerticalCol(props) {
    const stylesCol = {flexBasis: props.basis};
    return (
        <div className="vertical-col" style={stylesCol}>
            {props.children}
        </div>
    )
}

export default VerticalCol
