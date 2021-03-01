//
// Created by jbs on 21. 1. 22..
//
#include <utils/utility.h>

namespace px4_code2 {


    double getYaw (const geometry_msgs::Quaternion& q){
        tf::Quaternion qq;
        qq.setX(q.x);
        qq.setY(q.y);
        qq.setZ(q.z);
        qq.setW(q.w);
        qq.normalize();
        double r,p,y;
        tf::Matrix3x3 Q(qq); Q.getRPY(r,p,y);
        return y;
    }

    geometry_msgs::Quaternion getQuatFromYaw(double yaw){
        tf::Quaternion qq;
        qq.setRPY(0,0,yaw);
        qq.normalize();
        geometry_msgs::Quaternion quat;
        quat.x  = qq.getX();
        quat.y  = qq.getY();
        quat.z  = qq.getZ();
        quat.w  = qq.getW();
        return quat;
    }

    double interpolate(const vector<double>& xData,const vector<double>& yData, const double& x,bool extrapolate){

        int size = xData.size();
        int i = 0;                                                                  // find left end of interval for interpolation
        if ( x >= xData[size - 2] )                                                 // special case: beyond right end
        {
            i = size - 2;
        }
        else
        {
            while ( x > xData[i+1] ) i++;
        }
        double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
        if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
        {
            if ( x < xL ) {yR = yL; }
            if ( x > xR ) yL = yR;
        }

        double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient
        return yL + dydx * ( x - xL );                                              // linear interpolation
    }


    geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry &odom) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position = odom.pose.pose.position;
        poseStamped.pose.orientation = odom.pose.pose.orientation;
        poseStamped.header = odom.header;
        return poseStamped;
    }

    nav_msgs::Path convertTo(const vector<geometry_msgs::Point>& points){
        nav_msgs::Path path;
        for (auto pnt : points){
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position = pnt; poseStamped.pose.orientation.w = 1;
            path.poses.push_back(poseStamped);
        }
        return path;
    }
    double distance(const geometry_msgs::Point& pnt1, const geometry_msgs::Point& pnt2){
        return sqrt(pow(pnt1.x - pnt2.x,2) + pow(pnt1.y - pnt2.y,2) + pow(pnt1.z - pnt2.z,2));

    }

    /**
     * Generate quaternion from yaw when initialized
     */
    void Trajectory::generateQuat() {

        // Inspect jump in yaw angle
        for (int n = 1 ; n < yaws.size() ; n++){
            if (yaws[n] - yaws[n-1] > 1.8*M_PI)
                for (int m = n ; m < yaws.size() ; m++)
                    yaws[m] -= 2*M_PI;

            if (yaws[n] - yaws[n-1] < -1.8*M_PI)
                for (int m = n ; m < yaws.size() ; m++)
                    yaws[m] += 2*M_PI;
        }



        qxs.clear(); qxs.resize(yaws.size());
        qys.clear(); qys.resize(yaws.size());
        qzs.clear(); qzs.resize(yaws.size());
        qws.clear(); qws.resize(yaws.size());

        for(int n =0 ; n < yaws.size() ; n++){
//            cout << yaws[n] << endl;
           auto quat = getQuatFromYaw(yaws[n]);
           qxs[n] = (quat.x);
           qys[n] = (quat.y);
           qzs[n] = (quat.z);
           qws[n] = (quat.w);
        }



    }

    /**
     * Trajectory construction with fixed yaw
     * @param trajPtr
     * @param fixedYaw
     * @param duration
     */
    Trajectory::Trajectory(TrajGenObj *trajPtr,double fixedYaw, double duration) {
        int N = TRAJ_SAMPLE_PNT; //TODO
        double dt = duration / N;
        for (int n = 0; n <= N; n++) {
            double t = dt * n;
            TrajVector p = trajPtr->eval(t, 0);
            ts.push_back(t);
            xs.push_back(p(0));
            ys.push_back(p(1));
            zs.push_back(p(2));
            yaws.push_back(fixedYaw);
        }
        generateQuat();
    }

    /**
     * Trajectory construction with linearly varying yaw
     * @param trajPtr
     * @param yaw0
     * @param yawf
     * @param duration
     */
    Trajectory::Trajectory(TrajGenObj *trajPtr,double yaw0, double yawf, double duration) {
        int N = TRAJ_SAMPLE_PNT; //TODO
        double dt = duration/N;

        for (int n = 0 ; n <= N ; n++){
            double t = dt*n;
            TrajVector p = trajPtr->eval(t,0);
            ts.push_back(t);
            xs.push_back(p(0));
            ys.push_back(p(1));
            zs.push_back(p(2));
            yaws.push_back(yaw0 + (yawf - yaw0)/N * n );
        }
        generateQuat();
    }


    /**
     * Trajectory construction with tangential yaw
     * @param trajPtr
     * @param duration
     */
    Trajectory::Trajectory(TrajGenObj *trajPtr, double duration) {
        int N = TRAJ_SAMPLE_PNT; //TODO
        double dt = duration/N;
        for (int n = 0 ; n <= N ; n++){
            double t = dt*n;
            TrajVector p = trajPtr->eval(t,0);
            ts.push_back(t);
            xs.push_back(p(0));
            ys.push_back(p(1));
            zs.push_back(p(2));

            TrajVector pdot = trajPtr->eval(max(t,1e-3),1); // TODO. zero evaluation with velocity  is numerical error...
            double yaw = atan2(pdot(1),pdot(0));
            yaws.push_back(yaw);
        }
        generateQuat();
    }
    /**
     * Trajectory construction from txt file
     * @param fileDir
     * @param isLoaded
     */
    Trajectory::Trajectory(string fileDir,bool& isLoaded) {
        ifstream file; file.open(fileDir);
        if (file.is_open()){
            while (!file.eof()){
                string line;
                getline(file,line);
                stringstream ss(line);
                std::string val;
                while (!ss.eof()){
                    getline(ss,val,' ');
                    ts.push_back(atof(val.c_str()));
                    getline(ss,val,' ');
                    xs.push_back(atof(val.c_str()));
                    getline(ss,val,' ');
                    ys.push_back(atof(val.c_str()));
                    getline(ss,val,' ');
                    zs.push_back(atof(val.c_str()));
                    getline(ss,val,' ');
                    yaws.push_back(atof(val.c_str()));
                }

            }

            ts.pop_back(); xs.pop_back();  ys.pop_back(); zs.pop_back(); yaws.pop_back();
            generateQuat();
            ROS_INFO("Trajectory constructed with duration = %f / pnt number = %d",ts.back(),ts.size());
            isLoaded = ts.size() > 1 ;

        }else{
            ROS_ERROR_STREAM(fileDir + " was not opened! Trajectory object will not be created.");
            isLoaded = false;
        }

    }

    Trajectory::Trajectory(const px4_code2_msgs::UploadTrajectoryRequest &req) {
        ts = req.ts;
        xs = req.xs;
        ys = req.ys;
        zs = req.zs;
        yaws = req.yaws;
        generateQuat();
    }


    void Trajectory::append(const Trajectory &otherTraj_) {

        Trajectory otherTraj = otherTraj_;
        for (auto & it : otherTraj.ts){
            it += ts.back();
        }

        ts.insert(ts.end(),otherTraj.ts.begin(),otherTraj.ts.end());
        xs.insert(xs.end(),otherTraj.xs.begin(),otherTraj.xs.end());
        ys.insert(ys.end(),otherTraj.ys.begin(),otherTraj.ys.end());
        zs.insert(zs.end(),otherTraj.zs.begin(),otherTraj.zs.end());
        yaws.insert(yaws.end(),otherTraj.yaws.begin(),otherTraj.yaws.end());
        generateQuat();
    }


    nav_msgs::Path Trajectory::getPath(string frameId) {
       nav_msgs::Path path; path.header.frame_id = frameId;
       int N = ts.size();
       for (int n = 0 ; n < N ; n ++ ){
           geometry_msgs::PoseStamped poseStamped;
           poseStamped.pose.position.x = xs[n];
           poseStamped.pose.position.y = ys[n];
           poseStamped.pose.position.z = zs[n];

           tf::Quaternion q;
           q.setRPY(0,0,yaws[n]); q.normalize();

           poseStamped.pose.orientation.x = q.x();
           poseStamped.pose.orientation.y = q.y();
           poseStamped.pose.orientation.z = q.z();
           poseStamped.pose.orientation.w = q.w();
           path.poses.push_back(poseStamped);
       }
       return path;
    }

    bool Trajectory::writeTxt(string fileDir) {
        ofstream file; file.open(fileDir);
        if(file.is_open()){
            for (int n = 0 ; n < ts.size() ; n++){
                file<<ts[n]<<" " << xs[n] <<" " <<ys[n] << " " << zs[n] << " " << yaws[n] << endl;
            }
            file.close();
            return true;
        }else{
            return false;
        }
    }

    void Mission::loadTrajectory(const Trajectory &traj) {

        missionTraj = traj; triggerTime = ros::Time::now();
        path.header.frame_id = frame;
        path.poses.clear();

        for (int n = 0 ; n < traj.ts.size() ; n++){
            path.poses.push_back(getCurDesPose(traj.ts[n]));
        }


    }

    geometry_msgs::PoseStamped Mission::getCurDesPose(double t) {
        lastEmitTime = t;
        bool extrapolate = false;
        if (t > missionTraj.ts.back()){
            ROS_WARN_THROTTLE(3,"Time evaluation %f of trajectory exceed limit %f. Clamped", t, missionTraj.ts.back());
            t = missionTraj.ts.back() - 0.0001;
        }

        if (t < missionTraj.ts.front()){
            ROS_WARN_THROTTLE(3,"Time evaluation %f of trajectory is too early %f. Clamped", t, missionTraj.ts.front());
            t = missionTraj.ts.front() + 0.0001;
        }
        double x= interpolate(missionTraj.ts,missionTraj.xs,t,extrapolate);
        double y= interpolate(missionTraj.ts,missionTraj.ys,t,extrapolate);
        double z= interpolate(missionTraj.ts,missionTraj.zs,t,extrapolate);

        double qx = interpolate(missionTraj.ts,missionTraj.qxs,t,extrapolate);
        double qy = interpolate(missionTraj.ts,missionTraj.qys,t,extrapolate);
        double qz = interpolate(missionTraj.ts,missionTraj.qzs,t,extrapolate);
        double qw = interpolate(missionTraj.ts,missionTraj.qws,t,extrapolate);

        tf::Quaternion q;
        q.setX(qx); q.setY(qy); q.setZ(qz); q.setW(qw); q.normalize();
//        cout << q.x() << " , " << q.y() << " , " << q.z() << " , " << q.w() << endl;

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = frame;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.pose.position.x = x;
        poseStamped.pose.position.y = y;
        poseStamped.pose.position.z = z;

        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();
        return poseStamped;
    }

    geometry_msgs::PoseStamped Mission::emitDesPose(ros::Time t) {
        if (isActive) {
            double tEval = emittedDuration + (t - triggerTime).toSec();
            lastEmitPose = getCurDesPose(tEval);
        }
        return lastEmitPose;
    }


}