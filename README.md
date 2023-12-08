# ROS2 AI Rover Personal Project

- OnShape Design Files: [Rover](https://cad.onshape.com/documents/681d339b3385a03b7c82fd40/w/77c29d5f0b7b6c4892b4e7f8/e/d86bbe048173de173ee3c9b3?renderMode=0&uiState=6573754ecfb8225d4675653b)

- Submodule: [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)

- Installed Binary Packages:
- ROS2 Foxy joy
- ROS2 Foxy depthai_ros

Notes:
- After building a new package:
```
source ~/ros2_rover/install/setup.bash
```

Current Status:
- BLE controller connected to Jetson
- PCA9685 node subscribed to /cmd_vel and /joy

TODOs:
- Decide on structure for throttle and steering absolute position topics
    - /joy should be absolute position
    - /cmd_vel should be velocity
    - new topic for posting absolute position needed
        - add PCA9685 node as another subscriber
- [Test depthai_ros2 example for RGB](https://github.com/luxonis/depthai-ros/blob/humble/depthai_examples/src/rgb_video_subscriber.cpp)
- Attempt to debug Rviz2:
```bash
[rviz2-4] libGL error: No matching fbConfigs or visuals found
[rviz2-4] libGL error: failed to load driver: swrast
[rviz2-4] libGL error: No matching fbConfigs or visuals found
[rviz2-4] libGL error: failed to load driver: swrast
[rviz2-4] libGL error: No matching fbConfigs or visuals found
[rviz2-4] libGL error: failed to load driver: swrast
[rviz2-4] [ERROR] [1701502432.347972099] [rviz2]: Failed to create an OpenGL context. BadValue (integer parameter out of range for operation)
[rviz2-4] [ERROR] [1701502432.348261964] [rviz2]: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at /tmp/binarydeb/ros-foxy-rviz-ogre-vendor-8.2.8/.obj-aarch64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
[rviz2-4] [ERROR] [1701502432.348650199] [rviz2]: rviz::RenderSystem: error creating render window: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at /tmp/binarydeb/ros-foxy-rviz-ogre-vendor-8.2.8/.obj-aarch64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
```

Future Plans:
- Add Llava 1.5 multimodal NLP + vision model
    - [Llava 1.5](https://arxiv.org/abs/2310.03744)
    - [Do As I Can, Not As I Say: Grounding Language in Robotic Affordances](https://arxiv.org/abs/2204.01691)
    - [ProgPrompt: Generating Situated Robot Task Plans using Large Language Models](https://arxiv.org/abs/2209.11302)
    - [Nvidia Eureka](https://arxiv.org/abs/2310.12931)


![Rover_v4](/imgs/rover_v4.jpeg)