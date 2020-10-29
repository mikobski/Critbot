import './VerticalContainer.scss';

function VerticalContainer(props) {
    return (
        <div className="vertical-container">
            {props.children}
        </div>
    )
}

export default VerticalContainer
