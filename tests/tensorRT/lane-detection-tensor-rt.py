import torch
import tensorrt as trt
import numpy as np
import cv2

import argparse
import os, time


if __name__ == "__main__":
    # Parse model path
    parser = argparse.ArgumentParser(description="TensorRT Lane Detection Test")
    parser.add_argument("--model", type=str, required=True, help="Path to the TensorRT engine file")
    parser.add_argument("--video", type=str, required=True, help="Path to the video file")
    args = parser.parse_args()
    model_path = args.model
    video_path = args.video
    # Load the TensorRT engine
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    
    if not os.path.exists("./tests/tensorRT/engine/sample.engine"):
        builder = trt.Builder(TRT_LOGGER)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, TRT_LOGGER)
        success = parser.parse_from_file(model_path)
        error_detected : bool = False
        for idx in range(parser.num_errors):
            print(parser.get_error(idx))
            error_detected = True
        assert not error_detected, "Error(s) detected while parsing the ONNX model. Check the logs for details."
        
        print("Successfully parsed the ONNX model.")
        
        # Build engine
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1 GB
        serialized_engine = builder.build_serialized_network(network, config)
        with open("./tests/tensorRT/engine/sample.engine", "wb") as f:
            f.write(serialized_engine)
    else:
        print("Engine file already exists. Skipping engine serialization.")
        
    print("Successfully built and serialized the TensorRT engine.")

    # Deserialize a plan
    runtime = trt.Runtime(TRT_LOGGER)
    with open("./tests/tensorRT/engine/sample.engine", "rb") as f:
        engine_data = f.read()
    engine = runtime.deserialize_cuda_engine(engine_data)
    print("Successfully deserialized the TensorRT engine.")
    
    context = engine.create_execution_context()
    print("Successfully created execution context.")
    IMAGE_W = 0
    IMAGE_H = 0
    
    # Query tensor info (run once to verify names/shapes)
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        print(name, engine.get_tensor_shape(name), engine.get_tensor_mode(name))
        if name == "image":
            IMAGE_W = engine.get_tensor_shape(name)[3]
            IMAGE_H = engine.get_tensor_shape(name)[2]

    # Allocate buffers
    input_shape = engine.get_tensor_shape("image")
    output_shape = engine.get_tensor_shape("logits")

    # Replace dynamic batch dim (-1) with 1
    input_shape_concrete  = tuple(d if d > 0 else 1 for d in input_shape)
    output_shape_concrete = tuple(d if d > 0 else 1 for d in output_shape)

    input_tensor  = torch.zeros(input_shape_concrete,  dtype=torch.float32, device="cuda")
    output_tensor = torch.zeros(output_shape_concrete, dtype=torch.float32, device="cuda")

    context.set_tensor_address("image",  input_tensor.data_ptr())
    context.set_tensor_address("logits", output_tensor.data_ptr())
    
    # Run inference on video
    stream = torch.cuda.Stream()
    
    cap = cv2.VideoCapture(video_path)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            cap.release()
            cap = cv2.VideoCapture(video_path)  # Restart video
            continue
            break
        
        resized = cv2.resize(frame, (IMAGE_W, IMAGE_H))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        chw     = np.transpose(normalized, (2, 0, 1))          # HWC -> CHW
        batch   = np.expand_dims(chw, axis=0)
        
         # Copy to GPU
        input_tensor.copy_(torch.from_numpy(batch))
        
        # Inference
        with torch.cuda.stream(stream):
            context.execute_async_v3(stream.cuda_stream)
        torch.cuda.synchronize()

        # output_tensor now has results — shape depends on your model
        result = output_tensor.cpu().numpy()
        lane_mask = (result[0, 0] > 0.5).astype(np.uint8) * 255
        # mask = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        # lane_mask_rgb = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        # lane_mask_rgb[:, :, 0] = 0  # Blue channel
        # lane_mask_rgb[:, :, 1] = lane_mask_rgb[:, :, 1]  # Green channel (keep)
        # lane_mask_rgb[:, :, 2] = 0  # Red channel
        # result = cv2.addWeighted(resized, 0.7, lane_mask_rgb, 0.3, 0)
        # result = cv2.resize(result, (frame.shape[1], frame.shape[0]))
        
        lane_mask_color = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)

        M = cv2.moments(lane_mask, binaryImage=True)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(lane_mask_color, (cx, cy), 8, (0, 0, 255), -1)
            
            # Draw line to edge of the mask
            cx = lane_mask.shape[1] // 2
            cy = lane_mask.shape[0] - 1

            row = lane_mask[cy, :]

            # Find left and right edges at this row
            left_edge  = np.argmax(row > 0)                        # first white pixel from left
            right_edge = len(row) - np.argmax(row[::-1] > 0) - 1  # first white pixel from right

            if row[left_edge] == 255:
                cv2.line(lane_mask_color, (cx, cy), (left_edge, cy),  (255, 0, 0), 2)
            if row[right_edge] == 255:
                cv2.line(lane_mask_color, (cx, cy), (right_edge, cy), (0, 255, 0), 2)

            cv2.circle(lane_mask_color, (cx, cy), 8, (0, 0, 255), -1)
        cv2.imshow("Lane Detection", lane_mask_color)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.03)
        
        
    cap.release()
    cv2.destroyAllWindows()
