# Udacity-CarND-Path-Planning

## Path Planning Project
The goal of this project is to safely and optimally travel in a busy highway.The Path and behavioral planner must show the following results

* Be at the speed limit of 50 miles per hr

* Total acceleration should not exceed 10 m/s^2

* The car must not collide with any vehicle either during lane following or lane changing phase.

* The total jerk should not exceed 10 m/s^3

* Car must optimally change lanes such that it completes the track as fast as possible.


## Rubric Points
*Compilations -> done
*atleast 4.32 miles covered -> done
*Car drives within speed limit -> done
*Jerk and acceleration safety limit not exceeded -> done
*No collisions -> done
*Car stays in lane -> done
*Chnage lane behaviour -> done
*Reflection -> done



## Model Documentation

The similator sends two key information - sensorfusion of ego vehicle and other participant vehicle . We also have a map of trajectory in separate txt file which we use as reference path for the car to follow. We do two things in first part we try to generate a smooth trajectory based on the waypoints car covered and our known global trajector. This smooth trajectory generator can account for lane change as well . The we write a FSM logic to help the car decided when to follow lane or change lane depending upon the sensor fusion data of other vechiles surrounding it .

###Smooth trajectory generator.
To generate a smooth trajectory we first see wether there are any waypoints left in previous path . If yes we add that as well , then we select three waypoints that are 30,60,90 S units from the car . depending upon lane choosen the d unit will vary . We then use a c++ spline implementation to fit a spline over these way points. We keep a limit on the size of waypoints to 50 points at any instance.So if there are any old way points we use that and fill rest with waypoints with our generated spline object.


### safe to change lane test.
Before we change lane we should see if our candidate lane is safe to take . This done by simple logic , we first filter all cars from a list of sensor fusion data based on their presence in our lane of interest.Then based on the time taken for our ego car to reach final waypoint , we project all the cars in lane of interest to their future position to see if they are colliding from front(30) as well as back(20).If there is no possible collision in future project then lane is safe to take.


'''
bool lane_safe(json sensor_fusion,int lane,double car_s,int prev_size,bool& safe,double& cost)
{ cost = 0;
  safe = true;
  for(int i =0;i<sensor_fusion.size();i++)
          {
            float d = sensor_fusion[i][6];
            if(d<(2+4*lane+2)&&d>(2+4*lane-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s+= ((double)prev_size*0.02*check_speed);
              if((check_car_s>car_s) && ((check_car_s-car_s)>30))
              {
                safe &= true;
                cost = check_car_s-car_s;
                
                
              }
            
              else if((check_car_s<car_s) && ((-check_car_s+car_s)>20))
              {
                safe &= true;
                cost = -check_car_s+car_s;
                
                
              }
              else
              {
                safe &=false;
              }

            }
          }
  return safe;
  
}


'''




### Behaviour Planning 


Our FSM follow three transition - 

*If in middle lane check if left or right lane is free.If both are free chose the lane with largest free distance.
*If lane is in left/right then see if lane in middle is safe to take . if yes change lane to centre.
*If no safe lane exists to change slow the speed of the car without exceeding jerk.



### Finite state machine logic for switching Lanes
```
if((check_car_s>car_s) && ((check_car_s-car_s)<30))
              {
                too_close = true;
                bool safe;
                double cost;
                
                if(lane == 1) 
                { vector<double> lane_cost {0,0,0};
                  int lane_stay =1;
                  if(lane_safe(sensor_fusion,0,car_s,prev_size,safe,cost))
                  { std::cout<<cost<<"left"<<std::endl;
                    lane_cost[0] = cost;
                    lane_stay =0;
                    
                  }
                  
                  if(lane_safe(sensor_fusion,2,car_s,prev_size,safe,cost))
                  { std::cout<<cost<<"right"<<std::endl;
                    lane_cost[2] = cost;
                    lane_stay =0;
                    
                  }

                  if(lane_stay ==1)
                  {
                    lane =lane;
                  }
                  else
                  { 
                    auto maxi = std::max_element(lane_cost.begin(),lane_cost.end());
                    lane = std::distance(lane_cost.begin(), maxi);

                  }
                  
                }
                else 
                {
                  if(lane_safe(sensor_fusion,1,car_s,prev_size,safe,cost))
                  { 
                      lane = 1;
                  }
                  

                }
                
              } 
```


## Result
The car was able to drive atleast 18 miles without any incidence
