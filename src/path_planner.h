#ifndef PP_H
#define PP_H

#include <vector>
#include <math.h>
#include <sys/types.h>
#include "utils.h"

using namespace std;

// The max s value before wrapping around the track back to 0
static const double MAX_S = 6945.554;

static const double DTIME = 0.02;
static const double MAX_VEL = 49.0 * 0.44704;  // reference speed in meter/sec
static const double S_RANGE_ALERT = 30.0;
static const double S_RANGE_DANGER = 10.0;
static const double LANE_WIDTH = 4.0;
static const double LANE_HWIDTH = 2.0;
static const double LANE_CLEARANCE_FWD = 30.0;
static const double LANE_CLEARANCE_BACK = 6.0;
static const double A_ACCEL = 0.08;
static const double A_BRAKE = -0.08;


class Car {

  public:

    int id_;
    double x_, y_, vx_, vy_;
    double speed_, yaw_;
    double s_, d_;
    uint lane_;
    double s_pred_;

    double target_speed_;

    Car(int id, double x, double y, double vx, double vy, double s, double d)
      : id_(id), x_(x), y_(y), vx_(vx), vy_(vy), s_(s), d_(d) {
      lane_ = (uint)round((d_ - LANE_HWIDTH) / LANE_WIDTH);
    }

    void updateEgoCar(double x, double y, double s, double d, double yaw, double speed) {
      x_ = x;
      y_ = y;
      s_ = s;
      d_ = d;
      yaw_ = yaw;
      speed_ = speed;
      lane_ = (uint)round((d_ - LANE_HWIDTH) / LANE_WIDTH);
    }

    void updateTrafficCar(double x, double y, double vx, double vy, double s, double d) {
      x_ = x;
      y_ = y;
      vx_ = vx;
      vy_ = vy;
      s_ = s;
      d_ = d;
      speed_ = sqrt(pow(vx_, 2) + pow(vy_, 2));
      lane_ = (uint)round((d_ - LANE_HWIDTH) / LANE_WIDTH);
    }

    bool isClose(double toS, double limit) const {
      double s_limit = limit;
      return (fabs(s_ - toS) < s_limit || fabs(MAX_S-s_ - toS) < s_limit || fabs(s_ - MAX_S-toS) < s_limit);
    }
};

class PathPlanner {

  public:
    PathPlanner(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s, 
                const vector<double> &map_waypoints_dx, const vector<double> &map_waypoints_dy);

    virtual ~PathPlanner();

    void update(const vector<Car> &cars, const vector<double> &previous_path_x, const vector<double> &previous_path_y, double end_path_s, double end_path_d);
    void predictCars();
    void createPathSpline(vector<double> &next_x_vals, vector<double> &next_y_vals);
    void calculatePath(vector<double> &next_x_vals, vector<double> &next_y_vals);

  private:
    // set middle lane as starting lane
    int lane_;
    double ref_vel_;
    uint prev_path_size_;

    // target velocity, usually max speed or speed of car in front of us
    double target_vel_;

    bool braking_;

    // map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;

    // previous path from simulator
    vector<double> previous_path_x_;
    vector<double> previous_path_y_;
    double end_path_s_;
    double end_path_d_;

    // surrounding cars from sensor fusion
    vector<Car> cars_;
};

#endif /* PP_H */
