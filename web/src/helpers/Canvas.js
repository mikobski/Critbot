// Based on https://medium.com/@pdx.lucasm/canvas-with-react-js-32e133c05258
import React, { useRef, useEffect } from "react"

const Canvas = props => {

  const { onDraw, ...propsRest } = props;  
  const refCanvasContainer = useRef(null);
  const refCanvas = useRef(null);

  useEffect(() => {
    const canvasContainer = refCanvasContainer.current;
    const canvas = refCanvas.current;
    const context = canvas.getContext("2d");
    let frameCount = 0;
    let animationFrameId;

    function resizeCanvas(canvasContainer, canvas) {
      const containerDim = canvasContainer.getBoundingClientRect();
      const width = Math.floor(containerDim.width);
      const height = Math.floor(containerDim.height);
      if (canvas.width !== width || canvas.height !== height) {
        const { devicePixelRatio: ratio = 1 } = window;
        const context = canvas.getContext("2d");
        canvas.width = width * ratio;
        canvas.height = height * ratio;
        canvas.style.maxWidth = `${canvas.width}px`;
        canvas.style.maxHeight = `${canvas.height}px`;
        context.scale(1, 1);
        return true;
      }
      return false;
    };

    const render = () => {
      frameCount++;
      resizeCanvas(canvasContainer, canvas);
      const { width, height } = context.canvas;
      context.clearRect(0, 0, width, height);
      onDraw(context, frameCount);
      animationFrameId = window.requestAnimationFrame(render);
    }
    render()

    return () => {
      window.cancelAnimationFrame(animationFrameId);
    }
  }, [onDraw])

  const stylesCanvasCntainer = {
    height: "100%"
  };
  const stylesCanvas = {
    height: "100%",
    width: "100%"
  };

  return (
    <div ref={ refCanvasContainer } style={ stylesCanvasCntainer }>
      <canvas ref={ refCanvas } style={ stylesCanvas } { ...propsRest } />
    </div>
  );
}

export default Canvas;
