
---

# PID Controller



[//]: # (Image References)
[image1]: ./figures/speed_with_P.png
[image2]: ./figures/speed_with_PD.png
[image3]: ./figures/speed_with_PID.png
[image4]: ./figures/CTE_PID.png

[video1]: ./video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
## Reflection

### Describe the effect each of the P,I, D components had in your implementation

In project I used two PIDs to control the speed and position(offset to lane center) separately.

#### For speed controlling:
* P component adjust how quickly the vehicle reaches the target speed, as known as `response time`. As P increases 
the `response time` becomes short, but it will overshoot as P continues increasing till the speed oscillates.
* D component can damp the overshoot and oscillation. However set it too big will cause the controlling signal(Gain) 
to oscillate.
* I component remove the system bias error. `Figure 2` shows a permanent error to 0, `Figure 3`corrects the error by 
introducing the I component

`Figure 1` shows a P - Only component with overshooting. `Figure 2`add D component to damp some overshooting. `Figure 3`
correct the system bias.

![alt_text][image1]
Figure 1

![alt_text][image2]
Figure 2

![alt_text][image3]
Figure 3

#### For position controlling:

For position control it is necessary first to set a stable speed, because if speed changes, vehicle will travel different
distance within same time slot. This is actually a challenge I wanted to solve using a online tuning algorithm.

The PID components for position function the same as for speed, except position depends on speed and environment change 
 for position has more influence on choosing right parameters.
 
 I will explain in next point.

---


### Describe how the final hyperparameters were chosen

Choosing hyperparameters of PID for speed is relatively easy, as travelling direction has little effect to
speed, I assume only the road surface property will affect the transfer result of controll Gain to process result.

As above showing figures, I recorded the errors in a file and plotted with gnuplot tool.(see Gnuplot scripts 
./plot_speed_cte.gnu and ./plot_Y_cte.gnu). By manually tuning
the PID components I got the final parameters as :
```
* Kp_s = 0.9;
* Ki_s = 0.0002;
* Kd_s = 1.1;
```

There is one problem, as Gain is bounded with -1 to 1, the controlling variable `throttle` is saturated if error is big.
I.e. speed up from 0 to 30 mph, the accumulated error is big because Gain is cut to min/max. It will likely overshoot and
converge slowly. In real driving, speed up is cascaded, e.g. from 0 to 5 then 15 then 30 gradually, then different PID 
parameters are needed due to the min/max boundary.


For position PID, I used the same manual tuning method, but need to balance the stability and `response time`. 
* In case controlling speed, the speed is not affected by direction of travelling(or less affected), so the errors are eliminated 
 purely by PID control variable. 

* In case position, PID control eliminates the error but error is changed by travelling, like going around the curve. 
If PID converges quick enough, then that is not a problem, but it is not the case in this project. That is why error is 
not descreasing as expected but oscillates wildly. PID tries best to reduce the error but also needs to not oscillate both 
errors and controlling value (here try to avoid steering back and forth too frequently, but rather smoothly). `Figure 4 `
shows a plot between error and control signals.

![alt_text][image4]
Figure 4


#### Smoothing vs response time

The problem is if P component is too small, it may gradually steer the car in straight road but slide out of the road due
 to insufficient steering; if P component is too big, then it overshoots even in straight road. 

This problem get more severe as speed increased, this is effectively reducing the sample rate of control.
The final manually tuned parameters are following by speed set with 30 mph.

```
* Kp_s = 0.16;
* Ki_s = 0.0005;
* Kd_s = 4;
```

Here is a video clip for final behavior of controlled car.
Here's a [link to my video result](./video.mp4)

Another approaching than manual tuning is online auto-tuning based on a cost function. I tried using gradient descent to
 adjust the PID parameters against cost which is meas squared error. This working is still on-going.
 

#### a realistic strategy

Following human behavior, people driving slower when his car off the center or at curve. I tried to mimic this behavior by 
reducing speed when position error is increasing (calculate mean square error, MSE); and set high speed when MSE is small.
The PID components needs to be adjusted by applying `sample time` ratio. In my trial, it did not work well, especially when 
speed is high. This also makes sense due to reduced sample rate. Possible improvement is increasing sample rate. The simulator 
samples at ~ 50 miliseconds. 

