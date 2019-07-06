# **Path Planning Project**

## Yongtao Li

---

The goals of this project are the following:

* Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic
* A successful path planner will be able to keep inside its lane, avoid hitting other cars and pass slower moving traffic

[//]: # (Image References)

[image1]: ./pathplanningdiagram.png "projet diagram"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.

---

#### 1. Compiling

All the source code compile and run successfully on my local machine. Since I'm using Windows10, I chose to setup the environment to run Ubuntu Bash on Windows.

#### 2. Valid Trajectories

##### 2.1 The car is able to drive at least 4.32 miles without incident.

Yes, my car could drive at least 4.32 miles without incident. You could watch the following video which is using the my path planner.

<a href="https://youtu.be/Y4MoBYAKNdU
" target="_blank"><img src="http://img.youtube.com/vi/Y4MoBYAKNdU/0.jpg" 
alt="High Way Driving" width="240" height="180" border="10" /></a>

##### 2.2 The car drives according to the speed limit.

Yes, the car drives without exceeding the speed limit. It has been done by using a reference speed to target. The car is going to acclerate if not reaching the speed limit and is going to change lane or deacclerate if being too close with the car in front of it.

##### 2.3 Max Acceleration and Jerk are not Exceeded.

Yes, the car drives without excedding max accleration and jerk limit. It has been done by using the spline function to generate a very smooth trajectory. The trajectory always starts from previous path which ensures a smooth transition between each path planning loop.

##### 2.4 Car does not have collisions.

Yes, the car doesn't have collisions. The prediction is scanning all cars from sensor fusion data to detect any car is too close to my car. If the car in front of my car is too close, my car would either decelerate to stay safe or change lane if it's safe to do so.

##### 2.5 The car stays in its lane, except for the time between changing lanes.

Yes, the car stays in its lane all the time. Since we are using Frenet coordinate, during keeping lane state it's very easy to maintain in the lane with a constant d. When it's going to change lane, the spline generates a smooth line to the intended lane. Then the path planner will switch back to keep lane state right after a lane changing state.

##### 2.6 The car is able to change lanes.

Yes, the car is able to change lanes when it's safe and making sense to do so.

#### 3. Reflection

To summarize the model, there are three major parts as you could see from the following diagram. I started from the project walk-through and built on it.

##### 3.1 Prediction
I'm tracking two major things on each loop:

* is there any car too close in front and rear on each lane?
* how many cars are in front of my car on each lane?

The closeness check is to avoid collision and to change lane safely. If the car in front of my car is too close, my car could either slow down or change lane. When my car wants to change lane, it need to have enough space in front and rear to do so.

Checking cars ahead in each lane helps to determine if my car wants to switch lane or not. It doesn't make sense to go to lane with more cars ahead.

##### 3.2 Behavior Planning

I'm using a simplified Finite State Machine to manage the behavior. If the previous state is lane changing, the next loop must go back to keeping lane. If the previous state is keeping lane, the car could either change lane left or right depends which side has fewer cars and how safe to do so.

##### 3.3 Trajectory Generation

This part is mainly from the project walk-through in the classroom. It's using the spline function to generate a smooth transition, either staying in lane or changing lane.

![alt text][image1]


