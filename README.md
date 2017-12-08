### PID Vehicle Controller Project
##### David Rose


-------------------------
##### Overview 
Given a driving simulator with API access to vehicle controls as well as CTE (cross-track-error), or the angle off from center of the road, to gial is to create a PID (proportional–integral–derivative) controller that can maneuver the car around the track as fast as safely possible. Code will be written in C++ and will be looped through a JSON filed populated through a WebSocket API.

![PID Diagram](https://github.com/cipher982/PID-Control/blob/master/media/PIDforDummies_pid_simplified.png "PID Diagram")

##### Steps Involved:
- Understand methods of accessing and controlling the car.
- Convert the PID formula to C++ code and implement.
- Work in the PID methods to the main program and have it control the vehicle driving on the track.

##### Vehicle Control API
Using a WebSocket server, the C++ console program communicates with the driving simulator through a JSON file that takes in and reads out various telemetry variables. These include: CTE, speed, angle, throttle, and steering angle.

In this PID controller I will need to observe the CTE and speed, while pushing throttle (including braking for values < 0) and steering angle in an attempt to keep up the highest possible speed while staying safely within the lane. Below is the code for reading the telemetry into C++:



``` cpp 
// j[1] is the data JSON object
double cte = std::stod(j[1]["cte"].get<std::string>());
double speed = std::stod(j[1]["speed"].get<std::string>());
double angle = std::stod(j[1]["steering_angle"].get<std::string>());
double throttle = std::stod(j[1]["throttle"].get<std::string>());
```


##### PID Method
- **Proportional** - is just directly connected to the error output, always aiming to center or zero-out.
- **Integral** - is a combination of both the error AND the duration of the combined errors. It iterates over time and adds up the errors.
- **Differential** - multiplies the rate of change by the derivative, acting is a buffer. It can be thought of in operation as a shock absorber on a car, as it resists more when more force is applied (or error in this case). This effect creates a stability towards the ideal proportional output.

![PID Graph](https://github.com/cipher982/PID-Control/blob/master/media/PID%20Graph.png "PID Graph")

``` cpp
void PID::Init(double Kp, double Ki, double Kd) {
  // tau
  this->Kp = Kp;
  this->Kd = Ki;
  this->Ki = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  previous_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;  
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  cout << "-" << Kp << " * " << p_error << " - " << Kd << " * " << 
               d_error << " - " << Ki << " * " << i_error << "\tTotalError: " << 
               -Kp * p_error - Kd * d_error - Ki * i_error << endl;
               
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}
```



