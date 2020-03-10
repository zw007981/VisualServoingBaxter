# VisualServoingBaxter
Vision servoing based object following and obstacle avoidance using the Baxter robot.

**Contents**

* [Introduction](#Introduction)
* [Usage](#Usage)
* [Result](#Result)

## Introduction

A visual servoing control method enhanced by an obstacle avoidance strategy using potential based function. The solution require both relative bearing measurements, which acquire from the RGB-camera by color-based detection method through OpenCV. Eye-in-hand sensor arrangement can provide a relative high accuracy and avoid vision collision. We also validate our approach by implementation on the Baxter robot.

## Usage

1. Save this package into catkin_ws/src/.

2. Verify the robot is enabled.

``` 
rosrun baxter_tools enable_robot.py -e
```

3. Start the joint trajectory controller.

``` 
rosrun baxter_interface joint_trajectory_action_server.py --mode position
```

4. Open the camera on the left hand.

``` 
rosrun baxter_tools camera_control.py -o left_hand_camera -r 960x600
```

``` 
rosrun image_view image_view image:=/cameras/left_hand_camera/image
```

5. Run the tracking part.

``` 
rosrun VisualServoingBaxter tracing.py
```

6. Run the following part.

``` 
rosrun VisualServoingBaxter tracing.py
```

## Result

Trajectories of the object and the Baxter robot.

<img src="https://github.com/zw007981/VisualServoingBaxter/blob/master/trajectory1.png">

<img src="https://github.com/zw007981/VisualServoingBaxter/blob/master/trajectory2.png">

