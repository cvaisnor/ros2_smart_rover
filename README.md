# ROS2 AI Rover Personal Project

- Submodule: [RPLidar A2M12 Package](https://github.com/Slamtec/sllidar_ros2)

Notes:
- After building a new package:
```
source ~/ros2_rover/install/setup.bash
```

TODOs:
- Setup bluetooth controller connection
- Add joystick node as publisher to cmd_vel
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