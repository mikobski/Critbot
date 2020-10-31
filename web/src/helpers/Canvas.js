// Based on https://medium.com/@pdx.lucasm/canvas-with-react-js-32e133c05258
import React, { useRef, useEffect } from "react"

const Canvas = props => {

  const { draw, ...rest } = props
  const canvasRef = useRef(null)

  useEffect(() => {
    const canvas = canvasRef.current
    const context = canvas.getContext("2d");
    let frameCount = 0;
    let animationFrameId;

    function resizeCanvas(canvas) {
      const { width, height } = canvas.getBoundingClientRect();

      if (canvas.width !== width || canvas.height !== height) {
        const { devicePixelRatio: ratio = 1 } = window;
        const context = canvas.getContext("2d");
        canvas.width = width * ratio;
        canvas.height = height * ratio;
        context.scale(1, 1);
        return true;
      }
      return false;
    };

    const render = () => {
      frameCount++
      resizeCanvas(canvas);
      const { width, height } = context.canvas;
      context.clearRect(0, 0, width, height);
      draw(context, frameCount);
      animationFrameId = window.requestAnimationFrame(render);
    }

    render()

    return () => {
      window.cancelAnimationFrame(animationFrameId);
    }
  }, [draw])

  const stylesCanvasCntainer = {
    height: "100%"
  };
  const stylesCanvas = {
    height: "100%",
    width: "100%"
  };

  return (
    <div style={stylesCanvasCntainer}>
      <canvas ref={canvasRef} style={stylesCanvas} {...rest} />
    </div>
  );
}

export default Canvas