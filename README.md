# yolov8_segmentation_tracking_ros 
ROS package for real-time object detection and segmentation using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

- The `tracker_node` provides real-time object detection and segmentation on incoming ROS image messages using the Ultralytics YOLO model.

- roi_visualize provides a visual overlay of the ROI on the camera feed.

- control_bbox computes control commands based on detection outputs and drives the robot accordingly.

![Rviz Sim](misc/output.gif)

## Setup ⚙
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Nil69420/yolov8_segmentation_tracking_ros.git
$ python3 -m pip install -r yolov8_segmentation_tracking_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
**NOTE**: If you want to download KITTI datasets, remove `GIT_LFS_SKIP_SMUDGE=1` from the command line.
## Run 🚀

```
$ roslaunch yolov8_segmentation_tracking_ros tracker.launch debug:=true
```
**NOTE**: If the 3D bounding box is not displayed correctly, please consider using a lighter yolo model(`yolov8n.pt`) or increasing the `voxel_leaf_size`.

## `tracker_node`
### Params
- `yolo_model`: Pre-trained Weights.  
For yolov8, you can choose `yolov8*.pt`, `yolov8*-seg.pt`.

  
  See also: https://docs.ultralytics.com/models/
- `input_topic`: Topic name for input image.
- `result_topic`: Topic name of the custom message containing the 2D bounding box and the mask image.
- `result_image_topic`: Topic name of the image on which the detection and segmentation results are plotted.
- `conf_thres`: Confidence threshold below which boxes will be filtered out.
- `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
- `max_det`: Maximum number of boxes to keep after NMS.
- `tracker`: Tracking algorithms.
- `device`: Device to run the model on(e.g. cpu or cuda:0).
  ```xml
  <arg name="device" default="cpu"/> <!-- cpu -->
  ```
  ```xml
  <arg name="device" default="0"/> <!-- cuda:0 -->
  ```
- `classes`: List of class indices to consider.
  ```xml
  <arg name="classes" default="[0, 1]"/> <!-- person, bicycle -->
  ```
  See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
- `result_conf`:  Whether to plot the detection confidence score.
- `result_line_width`: Line width of the bounding boxes.
- `result_font_size`: Font size of the text.
- `result_labels`: Font to use for the text.
- `result_font`: Whether to plot the label of bounding boxes.
- `result_boxes`: Whether to plot the bounding boxes.
### Topics
- Subscribed Topics:
  - Image data from `input_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- Published Topics:
  - Plotted images to `result_image_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
  - Detected objects(2D bounding box, mask image) to `result_topic` parameter. (yolov8_segmentation_tracking_ros/YoloResult)
    ```
    std_msgs/Header header
    vision_msgs/Detection2DArray detections
    sensor_msgs/Image[] masks
    ```
## `tracker_with_cloud_node`
### Params
- `camera_info_topic`: Topic name for camera info.
- `lidar_topic`: Topic name for lidar.
- `yolo_result_topic`: Topic name of the custom message containing the 2D bounding box and the mask image.
- `yolo_3d_result_topic`: Topic name for 3D bounding box.
- `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
- `voxel_leaf_size`: Voxel size for pointcloud downsampling.
- `min_cluster_size`: Minimum number of points that a cluster needs to contain.
- `max_cluster_size`: Maximum number of points that a cluster needs to contain.
### Topics
- Subscribed Topics:
  - Camera info from `camera_info_topic` parameter. ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
  - Lidar data from `lidar_topic` parameter. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(2D bounding box, mask image) from `yolo_result_topic` parameter. (yolov8_segmentation_tracking_ros/YoloResult)
    ```
    std_msgs/Header header
    vision_msgs/Detection2DArray detections
    sensor_msgs/Image[] masks
    ```
- Published Topics:
  - Detected cloud points to `/detection_cloud` topic. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(3D bounding box) to `yolo_3d_result_topic` parameter. ([vision_msgs/Detection3DArray](http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html))
  - Visualization markers to `/detection_marker` topic. ([visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html))

---
### `roi_visualize`

This node provides a visual representation of a Region of Interest (ROI) over an image stream.

- **Purpose**:  
  Draws a polygonal ROI on incoming images and publishes the modified image.

- **Key Functionality**:
  - Subscribes to the input image topic (in this example, `/zed2i/zed_node/rgb/image_rect_color`).
  - Converts the ROS image to an OpenCV image.
  - Draws the ROI polygon defined by four points. The default ROI is given by:
    ```python
    self.roi = [[0.4, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]]
    ```
  - Publishes the annotated image on `/zed2i/zed_node/image_roi`.

- **Dynamic Reconfigure**:  
  The node integrates with ROS's dynamic reconfigure server to adjust the ROI parameters on the fly.

- **Code Overview**:
  - Uses `cv_bridge` to convert between ROS images and OpenCV images.
  - Applies `cv2.polylines` to draw the ROI polygon.

---
### `control_bbox`

This node provides a control mechanism based on bounding box detections.

- **Purpose**:  
  Receives YOLO detection results and computes control commands (using PID control) to adjust the robot's trajectory.

- **Key Functionality**:
  - Subscribes to the `/yolo_result` topic (custom message: `YoloResult`) containing detection data.
  - Extracts bounding box centers from detections.
  - Checks whether a detection falls within a defined ROI (in pixel coordinates).
  - Uses a PID controller (with parameters `Kp`, `Ki`, `Kd`) to compute angular corrections.
  - Publishes velocity commands to `/skid_steer/cmd_vel` (message type: `Twist`).
  - Supports navigation enable/disable via the `/navigation_control` topic (using `Int32` messages) and can reverse the PID control if needed.
  - Provides services to adjust the linear velocity:
    - `/set_linear_velocity`
    - `/decrease_linear_velocity`

- **Algorithm Highlights**:
  - **PID Control**:  
    Computes the error between the bounding box center and the image center, applies deadband filtering, and smooths the PID output with a low-pass filter.
  - **ROI Check**:  
    The ROI is defined based on the image dimensions. Only detections within this ROI are considered for control.
  - **Dynamic Behavior**:  
    The node can switch between normal and reversed control modes based on navigation commands.
