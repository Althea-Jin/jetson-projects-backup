#include "planner_interface.h"

namespace scan_planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc,double max_jerk)
    {
        pp_.max_vel_ = max_vel;
        pp_.max_acc_ = max_acc;
        pp_.max_jerk_ = max_jerk;
        pp_.feasibility_tolerance_ = 0.05;
        pp_.ctrl_pt_dist = 0.2;
        pp_.planning_horizen_ = 7.0;
    }

    void PlannerInterface::setGridMapPos(PathPoint& cur_pose)
    {
        cur_pose_  = cur_pose;
        current_position_[0] = cur_pose.x;
        current_position_[1] = cur_pose.y;
        current_position_[2] = cur_pose.z;
    }


    void PlannerInterface::applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d origin,double inflate_values)
    {
        std::cout << "x_size =" << x_size << " y_size =" << y_size << " z_size" << z_size << std::endl;
        std::cout << "resolution =" << resolution << std::endl;
        std::cout << "origin =" << origin << std::endl;
        std::cout << "inflate_values =" << inflate_values << std::endl;

        // 1. 配置局部地图和障碍物膨胀形状。
        plan_env::GridMapConfig config;
        config.resolution = resolution;                         // 米 / 体素
        config.local_map_size = Eigen::Vector3d(x_size,y_size, z_size);
        // Atom01 URDF 简化碰撞体零位包络约:
        //   x=[-0.108, 0.122], y=[-0.208, 0.208], z=[-0.669, 0.327] m
        //   XY 最大半径约 0.24 m。
        // 用双圆柱近似机器人身体 footprint，给脚/外壳/建图误差留余量。
        config.inflation_radius = 0.32;                  // XY 平面圆柱半径，等效宽度约 0.64 m
        config.inflation_z_up = 0.70;                    // 从障碍物体素向上膨胀高度
        config.inflation_z_down = 0.70;                  // 从障碍物体素向下膨胀高度
        config.double_cylinder_offset = 0.12;            // 前后圆柱中心偏移，等效长度约 0.88 m

        grid_map_ = std::make_shared<plan_env::GridMap>(config);


        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam();
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    }

    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        global_cloud_.clear();
        for(int i = 0; i < obstacle.size();i++)
        {
            Eigen::Vector3d obstacle_pos;
            obstacle_pos[0] = obstacle[i].x;
            obstacle_pos[1] = obstacle[i].y;
            obstacle_pos[2] = obstacle[i].z;
            global_cloud_.push_back(obstacle_pos);
        }
    }

    void PlannerInterface::makePlan()
    {
        if (_global_plan_traj_.size() < 4)
        {
            std::cerr << "[PlannerInterface] global path has only "
                      << _global_plan_traj_.size()
                      << " points, skip planning." << std::endl;
            _plan_traj_results_.clear();
            a_star_pathes_.clear();
            return;
        }

        grid_map_->updateLocalMap(current_position_, global_cloud_);

        Eigen::Vector3d start_pt;
        Eigen::Vector3d start_vel;
        Eigen::Vector3d start_acc;
        Eigen::Vector3d local_target_pt;
        Eigen::Vector3d local_target_vel;
        vector<Eigen::Vector3d> point_set;
        vector<Eigen::Vector3d> traj_pts;

        for(int i = 0; i< _global_plan_traj_.size();i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,_global_plan_traj_[i].z);
            point_set.push_back(plan_pt);
        }

        if (point_set.size() < 4)
        {
            std::cerr << "[PlannerInterface] B-spline point set has only "
                      << point_set.size()
                      << " points, skip planning." << std::endl;
            _plan_traj_results_.clear();
            a_star_pathes_.clear();
            return;
        }

        start_pt[0] = cur_pose_.x;
        start_pt[1] = cur_pose_.y;
        start_pt[2] = cur_pose_.z;
        
        local_target_pt[0] = _global_plan_traj_[_global_plan_traj_.size()-1].x;
        local_target_pt[1] = _global_plan_traj_[_global_plan_traj_.size()-1].y;
        local_target_pt[2] = _global_plan_traj_[_global_plan_traj_.size()-1].z;

        start_vel[0] = cos(cur_pose_.theta);  //根据实际需求修改接入
        start_vel[1] = sin(cur_pose_.theta);
        start_vel[2] = 0;

        local_target_vel[0] = 0;//根据实际需求修改接入
        local_target_vel[1] = 0;
        local_target_vel[2] = 0;

        start_acc[0] =  cos(cur_pose_.theta);   //根据实际需求修改接入
        start_acc[1] =  sin(cur_pose_.theta);
        start_acc[2] = 0;

        auto start = std::chrono::system_clock::now();

        bool plan_success = reboundReplan(start_pt,start_vel, start_acc,local_target_pt,local_target_vel, point_set);
        if (plan_success)
           getTraj();


        auto end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        printf("MotionPlanner Total Running Time: %ld  ms \n", elapsed.count());
    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }

    void PlannerInterface::getAStarPath(vector<vector<Eigen::Vector3d>>& a_star_path)
    {
        a_star_path = a_star_pathes_;
    }

    void PlannerInterface::getObstacles(std::vector<Eigen::Vector3d>& obstacle) const
    {
       obstacle = grid_map_->inflatedCloud();
    }


    bool PlannerInterface::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set)
    {
        vector<Eigen::Vector3d> start_end_derivatives;
        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(start_acc);

        applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

        // The current optimizer intentionally uses a planar (XY) gradient. Keep the
        // resulting trajectory on the real world-height reference instead of allowing
        // the unoptimized Z control points to collapse to zero.
        auto restore_z_reference = [&start_pt, &local_target_pt](Eigen::MatrixXd& control_points) {
            if (control_points.rows() < 3 || control_points.cols() == 0)
                return;

            const double denominator = control_points.cols() > 1
                                           ? static_cast<double>(control_points.cols() - 1)
                                           : 1.0;
            for (int i = 0; i < control_points.cols(); ++i)
            {
                const double ratio = static_cast<double>(i) / denominator;
                control_points(2, i) = start_pt(2) +
                                       ratio * (local_target_pt(2) - start_pt(2));
            }
        };

        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
        if (ctrl_pts.cols() < 4)
        {
            std::cerr << "[PlannerInterface] failed to parameterize B-spline, control point count: "
                      << ctrl_pts.cols() << std::endl;
            a_star_pathes_.clear();
            return false;
        }

        a_star_pathes_ = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        static int vis_id = 0;

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        restore_z_reference(ctrl_pts);
        cout << "first_optimize_step_success=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
            continous_failures_count_++;
            return false;
        }

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success)
            {
                restore_z_reference(optimal_control_points);
                pos = UniformBspline(optimal_control_points, 3, ts);
            }
        }

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            return false;
        }
    
        updateTrajInfo(pos);

        continous_failures_count_ = 0;

        return true;
    }

    bool PlannerInterface::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; 

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);
        if (ctrl_pts.cols() < 4 || ts <= 0.0)
        {
            std::cerr << "[PlannerInterface] refine reparameterization failed, control point count: "
                      << ctrl_pts.cols() << ", ts: " << ts << std::endl;
            return false;
        }

        traj = UniformBspline(ctrl_pts, 3, ts);

        const int refine_seg_num = ctrl_pts.cols() - 3;
        if (refine_seg_num <= 0)
        {
            std::cerr << "[PlannerInterface] invalid refine segment number: "
                      << refine_seg_num << std::endl;
            return false;
        }

        double t_step = traj.getTimeSum() / refine_seg_num;
        if (t_step <= 0.0)
        {
            std::cerr << "[PlannerInterface] invalid refine sample step: "
                      << t_step << std::endl;
            return false;
        }

        bspline_optimizer_rebound_->ref_pts_.clear();

        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void PlannerInterface::updateTrajInfo(const UniformBspline &position_traj)
    {
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void PlannerInterface::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                            Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;
        if (seg_num <= 0)
        {
            std::cerr << "[PlannerInterface] invalid segment number before reparameterization: "
                      << seg_num << std::endl;
            ctrl_pts.resize(0, 0);
            dt = 0.0;
            time_inc = 0.0;
            return;
        }

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;
        if (duration <= 0.0 || dt <= 0.0)
        {
            std::cerr << "[PlannerInterface] invalid duration/dt during reparameterization, duration: "
                      << duration << ", dt: " << dt << std::endl;
            ctrl_pts.resize(0, 0);
            return;
        }

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }
        if (point_set.size() < 4)
        {
            point_set.clear();
            for (int i = 0; i < 4; ++i)
            {
                const double sample_time = duration * static_cast<double>(i) / 3.0;
                point_set.push_back(bspline.evaluateDeBoorT(sample_time));
            }
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
        if (ctrl_pts.cols() < 4)
        {
            std::cerr << "[PlannerInterface] B-spline reparameterization produced too few control points: "
                      << ctrl_pts.cols() << std::endl;
            ctrl_pts.resize(0, 0);
        }
    }

    void PlannerInterface::getTraj()
    {
        auto info = &local_data_;

        scan_planner::Bspline bspline;
        bspline.order = 3;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());

        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());

        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        vector<scan_planner::UniformBspline> traj_;
        double traj_duration_;

        scan_planner::UniformBspline pos_traj(pos_pts, bspline.order, 0.1);
        pos_traj.setKnot(knots);
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());
        traj_.push_back(traj_[2].getDerivative());

        traj_duration_ = traj_[0].getTimeSum();


        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
        _plan_traj_results_.clear();
        for (double t_cur = 0; t_cur <= traj_duration_; t_cur += 0.1) 
        {
            pos = traj_[0].evaluateDeBoorT(t_cur);
            vel = traj_[1].evaluateDeBoorT(t_cur);
            acc = traj_[2].evaluateDeBoorT(t_cur);
            PathPoint tempPath;
            tempPath.x = pos(0);
            tempPath.y = pos(1);
            tempPath.z = pos(2);
            _plan_traj_results_.push_back(tempPath);
        }
    }

}
