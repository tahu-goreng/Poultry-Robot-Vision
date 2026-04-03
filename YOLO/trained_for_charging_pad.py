from inference import InferencePipeline
from inference.core.interfaces.stream.sinks import render_boxes

pipeline = InferencePipeline.init(
    model_id="yolo-ajrgg/charging-pad-detection-instant-2",
    video_reference="0", # 0 for webcam
    on_prediction=render_boxes, # Function to run after each prediction
)
pipeline.start()
pipeline.join()