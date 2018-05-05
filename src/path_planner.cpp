#include "path_planner.h"
#include "spline.h"

PathPlanner::PathPlanner(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s, 
                         const vector<double> &map_waypoints_dx, const vector<double> &map_waypoints_dy)
  : map_waypoints_x_(map_waypoints_x), map_waypoints_y_(map_waypoints_y), map_waypoints_s_(map_waypoints_s),
    map_waypoints_dx_(map_waypoints_dx), map_waypoints_dy_(map_waypoints_dy),
    lane_(1), ref_vel_(0.0) {}

PathPlanner::~PathPlanner() {}


void PathPlanner::update(const vector<Car> &cars, const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
    double end_path_s, double end_path_d) {
  cars_ = cars;
  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
  prev_path_size_ = previous_path_x_.size();
}


void PathPlanner::createPathSpline(vector<double> &next_x_vals, vector<double> &next_y_vals) {
  Car car = cars_[0];

  // anchor points for path generation
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states, either the current position or the end of previous path state
  double ref_x = car.x_;
  double ref_y = car.y_;
  double ref_yaw = deg2rad(cars_[0].yaw_);

  double prev_car_x, prev_car_y;
  if (prev_path_size_ < 2) {
    // previous car heading
    prev_car_x = car.x_ - cos(ref_yaw);
    prev_car_y = car.y_ - sin(ref_yaw);
  } else {
    // using end of previous path state
    prev_car_x = previous_path_x_[prev_path_size_-2];
    prev_car_y = previous_path_y_[prev_path_size_-2];
    ref_x = previous_path_x_[prev_path_size_-1];
    ref_y = previous_path_y_[prev_path_size_-1];
    ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
  }
  ptsx.push_back(prev_car_x);
  ptsx.push_back(ref_x);
  ptsy.push_back(prev_car_y);
  ptsy.push_back(ref_y);

  double sdist = (lane_ == cars_[0].lane_) ? 30 : 30;

  // some future points
  vector<double> next_1 = getXY(car.s_pred_ + 1*sdist, 2 + 4 * lane_, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_2 = getXY(car.s_pred_ + 2*sdist, 2 + 4 * lane_, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_3 = getXY(car.s_pred_ + 3*sdist, 2 + 4 * lane_, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  ptsx.push_back(next_1[0]);
  ptsx.push_back(next_2[0]);
  ptsx.push_back(next_3[0]);
  ptsy.push_back(next_1[1]);
  ptsy.push_back(next_2[1]);
  ptsy.push_back(next_3[1]);

  // transform from global coord anchor points to car-local waypoints
  for (uint i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  for (uint i = 0; i < prev_path_size_; ++i) {
    next_x_vals.push_back(previous_path_x_[i]);
    next_y_vals.push_back(previous_path_y_[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (ref_vel_ * DTIME);
  
  double x_add_on = 0;
  for (uint i = 1; i < 50 - prev_path_size_; i++) {
    // sample points on spline
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //transform from car-local coord
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}


void PathPlanner::predictCars() {
  for (auto& car : cars_) {
    if (prev_path_size_ == 0) {
      // if there is no previous path from simulator predicted s-values will be the same actually
      car.s_pred_ = car.s_;
    } else {
      // otherwise use the end path s-value and constant velocity model for traffic cars
      if (car.id_ == 0) {
        car.s_pred_ = end_path_s_;
      } else {
        car.s_pred_ = car.s_ + prev_path_size_ * DTIME * car.speed_;
      }
    }
  }
}


void PathPlanner::calculatePath(vector<double> &next_x_vals, vector<double> &next_y_vals) {

  predictCars();

  target_vel_ = MAX_VEL;
  braking_ = false;

  bool car_ahead = false;
  bool left_occupied = false;
  bool right_occupied = false;
  double front_target_speed = MAX_VEL;
  vector<double> left_dists, right_dists;
  for (auto& car : cars_) {
    if (car.id_ != 0) {
      // fill up values representing distances to cars further along the road
      if (car.lane_ - cars_[0].lane_ == -1 && (car.s_ - cars_[0].s_) > 0.0) {
        left_dists.push_back(car.s_ - cars_[0].s_);
      } else if (car.lane_ - cars_[0].lane_ == 1 && (car.s_ - cars_[0].s_) > 0.0) {
        right_dists.push_back(car.s_ - cars_[0].s_);
      }

      double car_dist = car.s_ - cars_[0].s_;
      double car_dist_pred = car.s_pred_ - cars_[0].s_pred_;

      if (car.lane_ == cars_[0].lane_ && 
         ((car.s_ > cars_[0].s_ && car_dist < S_RANGE_ALERT) ||
         (car.s_pred_ > cars_[0].s_pred_ && car_dist_pred < S_RANGE_ALERT))) {
        // car is ahead of us (either currently or predictedly) in close range
        car_ahead = true;
        if (front_target_speed > car.speed_) {
          front_target_speed = car.speed_;
        }
        if (car.isClose(cars_[0].s_pred_, S_RANGE_DANGER)) {
          // front car is dangerously close, explicit braking may be required to avoid incident
          braking_ = true;
        }
      } else if (car.lane_ - cars_[0].lane_ == -1 && 
                ((car_dist < LANE_CLEARANCE_FWD && (-car_dist < LANE_CLEARANCE_BACK)) || 
                (car_dist_pred < LANE_CLEARANCE_FWD && (-car_dist_pred < LANE_CLEARANCE_BACK)))) {
        // car is on the left, either currently or predictedly in buffer zone
        left_occupied = true;
      } else if (car.lane_ - cars_[0].lane_ == 1 && 
                ((car_dist < LANE_CLEARANCE_FWD && (-car_dist < LANE_CLEARANCE_BACK)) || 
                (car_dist_pred < LANE_CLEARANCE_FWD && (-car_dist_pred < LANE_CLEARANCE_BACK)))) {
        // car is on the right, either currently or predictedly in buffer zone
        right_occupied = true;
      }
    }
  }

  if (car_ahead && lane_ == cars_[0].lane_) {
    bool left_change_allowed = (!left_occupied && lane_ > 0);
    bool right_change_allowed = (!right_occupied && lane_ < 2);

    if (left_change_allowed && right_change_allowed) {
      // if both left and right lanes are free to go, decide upon available free lane distance forward
      double min_left = 10000.0;
      if (left_dists.size() > 0 ) {
        min_left = *std::min_element(left_dists.begin(), left_dists.end());
      }
      double min_right = 10000.0;
      if (right_dists.size() > 0 ) {
        min_right = *std::min_element(right_dists.begin(), right_dists.end());
      }
      if (min_left > min_right) {
        right_change_allowed = false;
      } else {
        left_change_allowed = false;
      }
    }

    if (left_change_allowed) {
      printf("PLANNED LANE CHANGE - LEFT \n");
      lane_--;
    } else if (right_change_allowed) {
      printf("PLANNED LANE CHANGE - RIGHT \n");
      lane_++;
    } else {
      // no lane change, target speed if either maximum value or speed of the car ahead
      target_vel_ = front_target_speed;
    }
  } 

  // accelerate to gradually meet target speed
  if ((target_vel_ - ref_vel_) > A_ACCEL && !braking_) {
    ref_vel_ += A_ACCEL;
  } else if ((target_vel_ - ref_vel_) < A_BRAKE || braking_) {
    ref_vel_ += A_BRAKE;
  }

  if (ref_vel_ > MAX_VEL) {
    ref_vel_ = MAX_VEL;
  }

  createPathSpline(next_x_vals, next_y_vals);
}
