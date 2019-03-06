
#include <navigation_features/nav_features.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>
#include <sys/time.h>


using namespace std;

features::NavFeatures::NavFeatures() {}

features::NavFeatures::NavFeatures(tf::TransformListener* tf, float size_x, float size_y, float res) 
{
	tf_listener_ = tf;
	size_x_ = size_x*2.0;  	//m
	size_y_ = size_y*2.0;  	//m	
	resolution_ = res;		//m/cell	
	max_planning_dist_ = sqrt((size_x_*size_x_)+(size_y_*size_y_));
	insc_radius_robot_ = 0.40;
	
	use_loss_func_ = false;
	demo_path_.clear();
	

	setParams();

}




features::NavFeatures::NavFeatures(tf::TransformListener* tf, vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, float res)
{
	
	tf_listener_ = tf;
	insc_radius_robot_ = insc_radius;
	size_x_ = size_x*2.0;  	//m
	size_y_ = size_y*2.0;  	//m	
	resolution_ = res;		//m/cell	
	max_planning_dist_ = sqrt((size_x_*size_x_)+(size_y_*size_y_));
	
	myfootprint_ = footprint;
	
	use_loss_func_ = false;
	demo_path_.clear();
	
	setParams();

 	
}


void features::NavFeatures::setParams()
{
	//Read the ROS params from the server
	ros::NodeHandle n("~/Navigation_features");


	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);
	
	
	//Load weights "w1, w2, ..."
	w_.clear();
	bool ok = true;
	unsigned int i = 1;
	while(ok)
	{
		char buf[10];
		sprintf(buf, "w%u", i);
		string st = string(buf);
				
		if(n.hasParam(st.c_str())){
			double wg = 0.0;
			n.getParam(st.c_str(), wg);
			w_.push_back((float)wg);	
			printf("NavFeatures. weight %u= %.3f loaded\n", i, wg);
		} else {
			//printf("param '%s' not found\n", st.c_str());
			ok = false;
		}
		i++;
	}
	
	n.param<int>("upo_featureset", upo_featureset_, 0);
	
	n.param<bool>("use_global_map", use_global_map_, false);
	
	string pc_topic;
	n.param<string>("pc_topic", pc_topic, string("/scan360/point_cloud")); 
	int pc_type;
	n.param<int>("pc_type", pc_type, 2); //1->PointCloud, 2->PointCloud2
	
		
	max_cost_obs_ = distance_functions(0.0, INVERSE_DEC); //EXP_DEC //INVERSE_DEC
	if(use_global_map_)
		setupMapProjection();
	else
		setupNoMapProjection();
	
	
	double front;
	n.param<double>("stddev_person_front", front, 1.2);
	double aside;
	n.param<double>("stddev_person_aside", aside, (1.2/1.5)); //=0.8
		
	double right;
	n.param<double>("stddev_person_right", right, 0.8); 
		
	n.param<bool>("enable_grouping", grouping_, true); 
	double aux;
	n.param<double>("stddev_group", aux, 0.8);
	stddev_group_ = (float)aux;
	double aux2;
	n.param<double>("grouping_distance", aux2, 1.5);
	grouping_distance_ = (float)aux2;
		
	amp_ = 1.80;			//Amplitude, max value of the gaussian
	sigmas_.push_back((float)front);  //front of person. front
	sigmas_.push_back((float)aside); //aside of person. front
	sigmas_.push_back(sigmas_.at(1)); //front of person. back
	sigmas_.push_back(sigmas_.at(1)); //aside of person. back
	sigmas_.push_back(right); //right of person. front
	sigmas_.push_back(right/2.5); //right of person. aside
	
	n.param<double>("approaching_angle", aux, 0.40);
	approaching_angle_ = (float)aux;
	
	
	it_remove_gauss_ = false;
	
	//Goal subscription
	//n.param<int>("goal_type", goal_type_, 1); //1->RRT, 2->A*
	//if(goal_type_ == 1)
		goal_sub_ = nh_.subscribe("/rrt_goal", 1, &NavFeatures::goalCallback, this);
	//else
		//goal_sub_ = nh_.subscribe("/move_base/current_goal", 1, &NavFeatures::goalAstarCallback, this);
	
	//People subscription
	sub_people_ = nh_.subscribe("/people/navigation", 1, &NavFeatures::peopleCallback, this); 

	//Obstacles subscription
	if(pc_type == 1) 	//pointCloud
		sub_pc_ = nh_.subscribe(pc_topic, 1, &NavFeatures::pcCallback, this);
	else   				//pointCloud2
		sub_pc_ = nh_.subscribe(pc_topic, 1, &NavFeatures::pc2Callback, this);
	
	
	pub_gaussian_markers_ = n.advertise<visualization_msgs::MarkerArray>("gaussian_markers", 5);
	
	ros::NodeHandle nd("navigation_features");
	//ros::ServiceServer service = nd.advertiseService("setApproachingIT", setApproachingIT);
	loss_srv_ = nd.advertiseService("set_use_loss_func", &features::NavFeatures::setLossService, this);
	valid_srv_ = nd.advertiseService("is_pose_valid", &features::NavFeatures::isPoseValidService, this);
	weights_srv_ = nd.advertiseService("setWeights", &features::NavFeatures::setWeightsService, this);
	init_weights_srv_ = nd.advertiseService("initWeights", &features::NavFeatures::initializeWeightsService, this);
	scenario_srv_ = nd.advertiseService("setScenario", &features::NavFeatures::setScenarioService, this);
	features_srv_ = nd.advertiseService("getPathFeatureCount", &features::NavFeatures::getFeatureCountService, this);


	//Dynamic reconfigure
	dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n); //ros::NodeHandle("~")
    dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
	
}


features::NavFeatures::~NavFeatures() {
	
}


 void features::NavFeatures::reconfigureCB(navigation_features::nav_featuresConfig &config, uint32_t level){

    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//configuration_mutex_.lock();
    
    upo_featureset_ = config.upo_featureset;
	//use_global_map_ = config.use_global_map;
	sigmas_[0] = config.stddev_person_front;
	sigmas_[1] = config.stddev_person_aside;
	sigmas_[2] = sigmas_[1];
	sigmas_[3] = sigmas_[1];
	sigmas_[4] = config.stddev_person_right;
	sigmas_[5] = sigmas_[4]/2.5;
	grouping_ = config.enable_grouping;
	stddev_group_ = config.stddev_group;
	grouping_distance_ = config.grouping_distance;
	it_id_ = config.interaction_target_id;
	it_remove_gauss_ = config.it_remove_gaussian;
	//printf("upo_featureset. it_id:%i\n", it_id_);

	//configuration_mutex_.unlock();
}


//Service
bool features::NavFeatures::setWeightsService(navigation_features::SetWeights::Request  &req, navigation_features::SetWeights::Response &res)
{
	setWeights(req.weights);
	return true;
}



//Service
bool features::NavFeatures::setLossService(navigation_features::SetLossCost::Request &req, navigation_features::SetLossCost::Response &res)
{
	printf("NavFeatures. Enabling loss function: %i\n", (int)req.use_loss_func);
	set_use_loss_func(req.use_loss_func); 
	set_demo_path(req.demo_path);

	return true;
}

//Service
bool features::NavFeatures::isPoseValidService(navigation_features::PoseValid::Request &req, navigation_features::PoseValid::Response &res)
{
	geometry_msgs::PoseStamped p = req.pose;
	res.ok = poseValid(&p);
}

//Service
bool features::NavFeatures::setScenarioService(navigation_features::SetScenario::Request &req, navigation_features::SetScenario::Response &res)
{
	setScenario(req.obstacles, req.people, req.goal); 

	return true;
}

//Service
bool features::NavFeatures::getFeatureCountService(navigation_features::GetFeatureCount::Request &req, navigation_features::GetFeatureCount::Response &res)
{ 
	res.fc = getPathFeatureCount(&req.path);
	return true;
}

//Service
bool features::NavFeatures::initializeWeightsService(navigation_features::InitWeights::Request &req, navigation_features::InitWeights::Response &res)
{
	vector<float> w;
	if(req.random) {
		srand(time(NULL));
		for(int i=0; i<w_.size(); i++){
			float v = rand() % 10 + 2;
			w.push_back(v);
		}
	} else {
		w.assign((int)w_.size(), 0.0);
	}

	//Normalize weights
	if(req.normalize && req.random) {	
		float total = accumulate(w.begin(), w.end(), 0);
		for(unsigned int i=0; i<w.size(); i++) 
			w[i] = w[i]/total;
	}
	w_ = w;
	res.weights = w;
	return true;
}




void features::NavFeatures::setScenario(sensor_msgs::PointCloud2 obs, upo_msgs::PersonPoseArrayUPO people, geometry_msgs::PoseStamped goal)
{
	setGoal(goal);
	setPeople(people);
	setObstacles(obs);
}



void features::NavFeatures::setObstacles(sensor_msgs::PointCloud2 obs)
{
	//printf("NavFeatures. Setting obstacles!\n");
	sensor_msgs::PointCloud2 lcloud;
	obs.header.stamp = ros::Time();
	try{  
		if(use_global_map_) {
			if(!pcl_ros::transformPointCloud("/map", obs, lcloud, *tf_listener_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		} else {
			if(!pcl_ros::transformPointCloud("/base_link", obs, lcloud, *tf_listener_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		}			

	} catch (tf::TransformException ex){
		ROS_WARN("NAV FEATURES. pcCallback. TransformException: %s", ex.what());
	}
	
	laserMutex_.lock();
	laser_cloud_ = lcloud;
	laserMutex_.unlock();
}




/**
 * Calculate the loss function
 * Input:
 	x, point coordinate
	y, point coordinate
	example_path, demonstration path
 * Output:
	loss_value in the range [0, 0.5]
 */
float features::NavFeatures::calculate_loss_function(geometry_msgs::PoseStamped* p)
{
	if(demo_path_.empty())
	{
		printf("\n	NavFeatures: ERRORRRRRRR! DEMONSTRATION PATH IS EMPTY!!!!!\n");
		return 0.0; 
	}
	if(!p) {
		printf("\n	NavFeatures: ERRORRRRRRR! point is null!!!!!\n");
		return 0.0;
	}
	
	geometry_msgs::PoseStamped point = *p;
	//printf("calculate loss function. header: %s\n", point.header.frame_id.c_str());
		
	if(point.header.frame_id.empty() || point.header.frame_id.length() < 3)
		printf("\n¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡calculate_loss_function. global_frame invalid!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
					
	//Just in case
	if(point.header.frame_id.compare("/odom") != 0 && point.header.frame_id.compare("odom") != 0)
	{
		//printf("calculate loss function. header: %s x:%.2f y:%.2f, th:%2f\n", point.header.frame_id.c_str(), point.pose.position.x, point.pose.position.y, tf::getYaw(point.pose.orientation));
		point = transformPoseTo(point, "odom", false); //true
	}
	float x = point.pose.position.x;
	float y = point.pose.position.y;
	
	
	float min_dist = 9999.0;
	for(unsigned int i=0; i<demo_path_.size(); i++)
	{
		geometry_msgs::PoseStamped ps = demo_path_[i];
		//Just in case
		if(ps.header.frame_id.compare("/odom") != 0 && ps.header.frame_id.compare("odom") != 0)
		{
			printf("calculate loss function. Demo path!! header: %s x:%.2f y:%.2f, th:%2f\n", ps.header.frame_id.c_str(), ps.pose.position.x, ps.pose.position.y, tf::getYaw(ps.pose.orientation));
			ps = transformPoseTo(ps, "odom", false); //true
		}
			
		float xe = ps.pose.position.x;
		float ye = ps.pose.position.y;
		float d = sqrt((x-xe)*(x-xe) + (y-ye)*(y-ye));
		if(d < min_dist)
			min_dist = d; 
	}
	
	if(min_dist > 1.0)
		min_dist = 1.0;

	return min_dist;
	
	//The cost decreases linearly from 1 to 0 in a distance of 1 meter
	//return (float)(1-min_dist)/2.0;
	
}






void features::NavFeatures::setupMapProjection()
{
	
	people_paint_area_ = 25;
	
	ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap> ("/static_map");
	while (! ros::service::waitForService("/static_map",1)){
		ROS_INFO("Waiting for map service");
	}

	nav_msgs::GetMap srv;
	map_client.call(srv);
	ROS_INFO_STREAM(srv.response.map.info);
	map_image_ = cv::Mat(srv.response.map.info.height, srv.response.map.info.width,CV_8UC1, cv::Scalar(0));
	map_metadata_ =srv.response.map.info;
	resolution_ = (double) map_metadata_.resolution;
	origin_.push_back(map_metadata_.origin.position.x); //m
	origin_.push_back(map_metadata_.origin.position.y); //m
	origin_.push_back(tf::getYaw(map_metadata_.origin.orientation));
	uint8_t *myData = map_image_.data;
	for (int i=0;i<srv.response.map.data.size();i++){
		if (srv.response.map.data.at(i)==100  || srv.response.map.data.at(i)==-1 ){
		}
		else {
			map_image_.data[i] = 255;
		}
	}

	dtMutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	dtMutex_.unlock();
 
}


void features::NavFeatures::setupNoMapProjection()
{
	people_paint_area_ = 25;
	
	map_image_ = cv::Mat((int)size_x_/resolution_, (int)size_y_/resolution_, CV_8UC1, cv::Scalar(255));
	origin_.push_back(size_x_/2.0);
	origin_.push_back(size_y_/2.0);
	origin_.push_back(0.0);

	map_metadata_.resolution = resolution_;
	map_metadata_.width = (int)size_x_/resolution_;   //Cells
	map_metadata_.height = (int)size_y_/resolution_; //Cells
	geometry_msgs::Pose orig;
	orig.position.x = origin_[0];
	orig.position.y = origin_[1];
	orig.position.z = 0.0;
	orig.orientation = tf::createQuaternionMsgFromYaw(origin_[2]);
	map_metadata_.origin = orig; //m

	dtMutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	//cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	dtMutex_.unlock();
 
}




void features::NavFeatures::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//printf("NavFeatures. Receiving goal!\n");
	setGoal(*msg);
}

/*void features::NavFeatures::goalAstarCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	setGoal(*msg);
}*/

void features::NavFeatures::peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg) 
{
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//printf("NavFeatures. Receiving people!\n");
	peopleMutex_.lock();
	people_ = msg->personPoses;
	if(people_.size() > 0) {
		people_frame_id_ = msg->header.frame_id;
	}
	peopleMutex_.unlock();

	
}


void features::NavFeatures::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	//printf("NavFeatures. Setting People!\n");
	peopleMutex_.lock();
    
    people_.clear();
    people_ = p.personPoses;
    people_frame_id_ = p.header.frame_id;
	
	peopleMutex_.unlock();
}





//Point_cloud2 callback
// Only used if laser projection is employed
void features::NavFeatures::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& pc_in){
	
	setObstacles(*pc_in);
}


//Point_cloud callback
// Only used if laser projection is employed
void features::NavFeatures::pcCallback(const sensor_msgs::PointCloud::ConstPtr& pc_in){
	
	sensor_msgs::PointCloud2 pc2;
	bool ok = sensor_msgs::convertPointCloudToPointCloud2(*pc_in, pc2);
	if(!ok) {
		ROS_WARN("NavFeatures. Error transforming pointCloud to pointCloud2");
	}
	
	setObstacles(pc2);
	
}





void features::NavFeatures::updateDistTransform(){
	
	sensor_msgs::PointCloud temp_pt_cloud;
	laserMutex_.lock();  
	sensor_msgs::PointCloud2 lcloud = laser_cloud_;
	laserMutex_.unlock();
	
	if(lcloud.data.size() <= 0) {
		ROS_WARN("No cloud updated");
		return;
	}
	
	bool done = sensor_msgs::convertPointCloud2ToPointCloud(lcloud, temp_pt_cloud);
	if(!done) 
		ROS_ERROR("\n\nNAV FEATURES. UPDATEDT. convertPointCloud2toPoingCloud!!!!!\n");
	
	//Add the laser readings (pointcloud) to the map
	dtMutex_.lock();
	cv::Mat map_copy = map_image_.clone();
	dtMutex_.unlock();
	for (int i =0;i<temp_pt_cloud.points.size();i++) {
		vector<int> pix;
		if(use_global_map_)
			pix = worldToMap(&(temp_pt_cloud.points[i]),&map_metadata_);
		else
			pix = BaseLinkWorldToImg(&(temp_pt_cloud.points[i]));

		if(pix[0] >= 0.0 && pix[0] < map_metadata_.width && pix[1] >= 0.0 && pix[1] < map_metadata_.height) 
			map_copy.at<unsigned char>(pix[1],pix[0]) = 0;	
		//else
			//ROS_WARN("\n\nNAV FEATURES. UPDATEDT. pixel out of range!\n");
	}

	
	//Remove the people detected from the map
	peopleMutex_.lock();
	vector<upo_msgs::PersonPoseUPO> per = people_;
	peopleMutex_.unlock();
	for (int i=0; i<per.size(); i++){
		geometry_msgs::Point32 temp_point;
		temp_point.x = per[i].position.x;
		temp_point.y = per[i].position.y;
		vector<int> pix;
		if(use_global_map_)
			pix = worldToMap(&temp_point,&map_metadata_);
		else
			pix = BaseLinkWorldToImg(&temp_point);
		
		if (pix[0] >= 0.0 && pix[0] < map_metadata_.width && pix[1] >= 0.0 && pix[1] < map_metadata_.height)
		{
			for (int j = -floor(people_paint_area_/2);j<ceil(people_paint_area_/2);j++){
				for (int k = -floor(people_paint_area_/2);k<ceil(people_paint_area_/2);k++){
					if (pix[1]+k>=0 && pix[0]+j>=0){
						map_copy.at<unsigned char>(pix[1]+k,pix[0]+j) = 255;
					}
				}

			}
		}
	}

	
	cv::Mat dt;
	try{
		//cv::distanceTransform(map_copy, distance_transform_, CV_DIST_L1,3);
		cv::distanceTransform(map_copy, dt, CV_DIST_L1, 3);
	} catch(...){
		ROS_ERROR("\n\nNAV FEATURES. UPDATEDT. cv::distanceTransform!!!!!\n");
	} 

	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	
	dtMutex_.lock();
	distance_transform_ = dt.clone();
	dtMutex_.unlock(); 
	

	//gettimeofday(&stop, NULL);
	//printf("took %lu\n", stop.tv_usec - start.tv_usec);
	//imwrite(ros::package::getPath("navigation_features").append("/test_write.jpg"), map_copy);
}



vector<int> features::NavFeatures::worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata){
	vector<int> pixels;
	float x_map = world_point->x - map_metadata->origin.position.x;
	float y_map = world_point->y - map_metadata->origin.position.y;
	pixels.push_back((int)floor(x_map/map_metadata->resolution ));
	pixels.push_back((int)floor(y_map/map_metadata->resolution));
	return pixels;

}

/**
* transform point in base link frame (m) to pixel
*/
vector<int> features::NavFeatures::BaseLinkWorldToImg(geometry_msgs::Point32* point)
{ 
	vector<int> pix;
	float wx = origin_[0] + point->x;
	float wy = origin_[1] + point->y; //-
	pix.push_back((int)floor(wx/resolution_));
	pix.push_back((int)floor(wy/resolution_));
	return pix;

}



void features::NavFeatures::setGoal(geometry_msgs::PoseStamped g) {

	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//printf("NavFeatures. Receiving goal point!\n");
	goal_ = g;
}





void features::NavFeatures::setWeights(vector<float> we) {
	printf("NavFeatures. Setting weights: \n");
	w_.clear();
	for(unsigned int i=0; i<we.size(); i++) 
	{
		//if(we[i] != 0.0) {
			w_.push_back(we[i]);
			//printf("we%u: %.3f, w_%u: %.3f\n", (i+1), we[i], (i+1), w_[i]);
		//}
	}
	
	for(unsigned int i=0; i<w_.size(); i++) 
	{
		printf("w_%u: %.3f\n", (i+1), w_[i]);
	}
	
	/*if(w_.size() == 3)
		upo_featureset_ = 0;
	else if(w_.size() == 4)
		upo_featureset_ = 2;
	else
		upo_featureset_ = 1;
	*/
}







bool features::NavFeatures::poseValid(geometry_msgs::PoseStamped* pose)
{
	
	//Transform the coordinates
	vector<int> pix;
	geometry_msgs::PoseStamped sm;
	if(use_global_map_) {
		sm = transformPoseTo(*pose, string("/map"), false);
		geometry_msgs::Point32 point;
		point.x = sm.pose.position.x;
		point.y = sm.pose.position.y;
		point.z = 0.0;
		pix = worldToMap(&point, &map_metadata_);
	}else {
		sm = transformPoseTo(*pose, string("/base_link"), false);
		geometry_msgs::Point32 point;
		point.x = sm.pose.position.x;
		point.y = sm.pose.position.y;
		point.z = 0.0;
		pix = BaseLinkWorldToImg(&point);
	}
	//Distances to origin	
	//float distance_x = sm.pose.position.x - (origin_)[0];
	//float distance_y = sm.pose.position.y - (origin_)[1];
	//float px = floor(distance_x/resolution_);
	//float py =floor(distance_y/resolution_);

	float px = pix[0];
	float py = pix[1];
	float distance = 0.0;
	dtMutex_.lock();
	if (py<0 || px<0  || px > map_image_.cols || py > map_image_.rows) {
		distance = 0.0;
	} else{
		try{
			
			distance = distance_transform_.at<float>(py,px)*resolution_;
			
		} catch(...)
		{
			ROS_ERROR("ERROR. POSE_VALID. px:%.2f, py:%.2f, distance:%.2f", px, py, distance);
			distance = 0.0;
		}
	}
	dtMutex_.unlock();
	// Take into account the robot radius 
	if(distance <= insc_radius_robot_)
		return false;
	else
		return true;
	
}



//calculate the cost of a ray-traced line
/*float features::NavFeatures::lineCost(int x0, int x1, int y0, int y1){
		
	float line_cost = 0.0;
	float point_cost = -1.0;

	for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
	{
		point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

		if(point_cost < 0)
			return -1;

		if(line_cost < point_cost)
			line_cost = point_cost;
	}
	return line_cost;
}*/

/*float features::NavFeatures::pointCost(int x, int y){
	unsigned char cost = costmap_local_->getCost(x, y);
	//if the cell is in an obstacle the path is invalid
	if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION){
		return -1.0;
	}
		return cost;
}*/







float features::NavFeatures::getCost(geometry_msgs::PoseStamped* s)
{
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	
	//configuration_mutex_.lock();
	
	
	vector<float> features;

	//if(upo_featureset_ != 2) { 
		//Goal distance cost
		float dist_cost = goalDistFeature(s);
	
		if(dist_cost > 1.0) {
			//printf("GetCost. Dist_cost out of range > 1. returning 1!!!\n");
			dist_cost = 1.0;
		}
		if(dist_cost < 0.0){
			printf("GetCost. Dist_cost out of range < 0. Returning 0!!!\n");
			dist_cost = 0.0;
		}	
		features.push_back(dist_cost);
	//}
	
	//Obstacle distance cost
	float obs_cost = obstacleDistFeature(s);
	features.push_back(obs_cost);
	if(obs_cost > 1.0 || obs_cost < 0.0) {
		printf("GetCost. Obs_cost out of range!!! = %.2f\n", obs_cost);
	}


	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	
	if(upo_featureset_ == 0) {
	
		//Proxemics cost
		float prox_cost = proxemicsFeature(s);
		if(prox_cost > 1.0 || prox_cost < 0.0) {
			printf("GetCost. Prox_cost out of range!!!\n");
			prox_cost = 0.0;
		}
		features.push_back(prox_cost);
		
		//printf("GoalCost: %.3f, ObsCost: %.3f, ProxCost: %.3f\n", dist_cost, obs_cost, prox_cost); 
		if(obs_cost == 1.0)
			return 1.0;

		//if(prox_cost == 0.0 && obs_cost != 0.0)
		//	return (0.5*dist_cost + 0.5*obs_cost);	
		
	} else {  //split the proxemics cost (featureset 1 and 2)
		vector<float> prox_costs = gaussianFeatures(s);
		for(unsigned int i=0; i<prox_costs.size(); i++)
			features.push_back(prox_costs[i]);
			
		if(obs_cost == 1.0)
			return 1.0;
	}

	//configuration_mutex_.unlock();	

	//if(prox_cost == 0.0 && obs_cost != 0.0)
	//	return (0.5*dist_cost + 0.5*obs_cost);	
	//if(obs_cost == 0.0 && prox_cost != 0.0)
	//	return (0.5*prox_cost + 0.5*dist_cost);
	//if(prox_cost == 0.0 && obs_cost == 0.0)
	//	return dist_cost;
	
	float cost = 0.0;
	for(unsigned int i=0; i<w_.size(); i++)
		cost += w_[i]*features[i];
	
	//This is used for learning algorithms (MMP and RTL)
	loss_mutex_.lock();
	if(use_loss_func_ && !demo_path_.empty())
	{
		float l = calculate_loss_function(s);
		l = (float)(1-l); // /2.0;
		cost += l;
		
		//Two options:
		// 1. Saturate value
		//if(cost > 1.0)
		//	cost = 1.0;
		// 2. Normalize
		cost = cost/2.0;
		
	}
	loss_mutex_.unlock();
	return cost;	
}



vector<float> features::NavFeatures::getFeatures(geometry_msgs::PoseStamped* s) 
{
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);

	//configuration_mutex_.lock();

	vector<float> features;
	
	//if(upo_featureset_ != 2) { //featureset 2 is for A*, so we don't need this feature
		//Goal distance cost
		float dist_cost = goalDistFeature(s);
		features.push_back(dist_cost);
	//}	
	//Obstacle distance cost
	float obs_cost = obstacleDistFeature(s);
	features.push_back(obs_cost);
	
	if(upo_featureset_ == 0) {
		//Proxemics cost
		float prox_cost = proxemicsFeature(s);
		features.push_back(prox_cost);
		
	} else { //split the proxemics cost (featureset 1 and 2)
		vector<float> prox_costs = gaussianFeatures(s);
		for(unsigned int i=0; i<prox_costs.size(); i++) {
			features.push_back(prox_costs[i]);
		}
		
	}

	//configuration_mutex_.unlock();
	
	return features;
}



vector<float> features::NavFeatures::getPathFeatureCount(vector<geometry_msgs::PoseStamped>* path)
{
	printf("NavFeatures. Request of path feature count...\n");
	vector<float> feature_counts;

	for(int i=0; i<path->size()-1; i++)
	{
		//We have to transform the coordinates to robot frame (/base_link)
		geometry_msgs::PoseStamped robot_pose = path->at(i);
		if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
			robot_pose = transformPoseTo(robot_pose, "/base_link", false);

		geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
		if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
			robot_pose2 = transformPoseTo(robot_pose2, "/base_link", false);

		double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
		double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
		double d = hypot(dx,dy);

		if(!isQuaternionValid(robot_pose.pose.orientation)) {
			robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
		}
			
		update();
			
		vector<float> features1 = getFeatures(&robot_pose);
		vector<float> features2 = getFeatures(&robot_pose2);

		if(i==0)
			feature_counts.assign(features1.size(), 0);
				
		for(unsigned int j=0; j<features1.size(); j++)
		{
			feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))*d/2.0;
		}
			
	  }
	  return feature_counts;
}



float features::NavFeatures::goalDistFeature(geometry_msgs::PoseStamped* s)  {
	
	//if we don't have a goal yet
	if(goal_.header.frame_id.empty())
			return (float) 0.5;
	
	//if the frame_id are different, transform s to goal frame
	geometry_msgs::PoseStamped p = transformPoseTo(*s, goal_.header.frame_id, false);
	float dx = goal_.pose.position.x - p.pose.position.x;
	float dy = goal_.pose.position.y - p.pose.position.y;
	float dist = (float)(sqrt(dx*dx + dy*dy)/max_planning_dist_);
	if(dist > 1.0) {
		//printf("NavFeatures. goal dist feature %.2f > 1.0. d: %.2f, max_dist: %.2f\n", dist, (float)sqrt(dx*dx + dy*dy), max_planning_dist_);
		dist = 1.0;
	}
	return dist;
}



float features::NavFeatures::obstacleDistFeature(geometry_msgs::PoseStamped* s)  {
	
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//configuration_mutex_.lock();

	//Transform the coordinates
	vector<int> pix;
	geometry_msgs::PoseStamped sm;
	if(use_global_map_) {
		sm = transformPoseTo(*s, string("/map"), false);
		geometry_msgs::Point32 point;
		point.x = sm.pose.position.x;
		point.y = sm.pose.position.y;
		point.z = 0.0;
		pix = worldToMap(&point, &map_metadata_);
	}else {
		sm = transformPoseTo(*s, string("/base_link"), false);
		geometry_msgs::Point32 point;
		point.x = sm.pose.position.x;
		point.y = sm.pose.position.y;
		point.z = 0.0;
		pix = BaseLinkWorldToImg(&point);
	}

	float px = pix[0];
	float py = pix[1];
		
	// This functions transforms position of people to pixels on map, gets distance and then converts to meters
	//float distance_x = sm.pose.position.x - (origin_)[0];
	//float distance_y = sm.pose.position.y - (origin_)[1];

	//float px = floor(distance_x/resolution_);
	//float py =distanceTransform->rows-floor(distance_y /resolution);
	//float py =floor(distance_y/resolution_);
	//cout<<"px:"<<px<<"  py:"<<py<<endl;
	float distance = 0.0;
	dtMutex_.lock();
	if (py<0 || px<0  || px > map_image_.cols || py > map_image_.rows) {
		distance = 0.0;
	} else{
		try{
			
			distance = distance_transform_.at<float>(py,px)*resolution_;
			
		} catch(...)
		{
			ROS_ERROR("ERROR. OBSTACLE FEATURE. px:%.2f, py:%.2f, distance:%.2f", px, py, distance);
			distance = 0.0;
		}
	}
	dtMutex_.unlock();
	//cout << "after distance_trans" << endl;
	// Take into account the robot radius 
	if(distance <= insc_radius_robot_)
		distance = 0.0;
	else
		distance = distance - insc_radius_robot_;
			
			
	float d = distance_functions(distance, INVERSE_DEC); //EXP_DEC //INVERSE_DEC
	//Normalize
	//printf("Distance: %.4f, Obs cost: %.4f, norm: %.4f\n", distance, d, (d/max_cost_obs_));
	d = d/max_cost_obs_;
	if(d > 1.0) {
		printf("NavFeatures. obstacle dist feature %.2f > 1.0\n", d);
		d = 1.0;
	}

	//configuration_mutex_.unlock();
	
	return d;
		
}



float features::NavFeatures::distance_functions(const float distance, const dist_type type){
	// different distance transformations between two points are defined here
	// _INC means the function is monotonically increasing
	// _DEC means that the function is monotonically decreasing
	float out=0;
	switch (type){
		case LINEAR_INC:	
			out = distance;
			break;
		case LOG_INC:   	
			out = 2*log(1+distance);
			break;
		case EXP_INC:   	
			out = exp((distance-2)/4-1);
			break;
		case INVERSE_DEC: 	
			out = 2/(distance+0.2);
			break;
		case LOG_DEC:     	
			out = 1/log(0.5*distance+1.2);
			break;
		case EXP_DEC:    	
			out = exp(3-3*distance);
			break;
		default:
			out=distance;
		break;
	}
	return out;
}



void features::NavFeatures::calculateGaussians()
{
	
	vector<upo_msgs::PersonPoseUPO> people_aux;
	string frame;
	//read the person list (thread safe)
	peopleMutex_.lock();
	people_aux = people_;
	frame = people_frame_id_;
	peopleMutex_.unlock();
	
	//printf("Frame %s\n", frame.c_str());
	//printf("\n\ncalculateGaussians. People detected: %u\n\n", (unsigned int)people_aux.size());
	
	vector<gaussian> gauss_aux;
	
	if(grouping_) 
	{
		
		vector<group_candidate> group_candidates;
		for(unsigned int i=0; i<people_aux.size(); i++)
		{
			//If the person is moving, we  do not group for the moment
			if(people_aux.at(i).vel > 0.18) {
				
				gaussian f;
				f.type = FRONT;
				f.x = people_aux.at(i).position.x;
				f.y = people_aux.at(i).position.y;
				f.th = tf::getYaw(people_aux.at(i).orientation);
				f.sx = sigmas_[0]; // 1.2
				f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
				
				if(people_aux.at(i).id == it_id_)
				{
					f.type = FRONT_APPROACH;
				}
				gauss_aux.push_back(f);
				
				gaussian b;
				b.type = BACK;
				b.x = people_aux.at(i).position.x;
				b.y = people_aux.at(i).position.y;
				float angle = (tf::getYaw(people_aux.at(i).orientation)) + M_PI;
				b.th = normalizeAngle(angle, -M_PI, M_PI);
				b.sx = sigmas_[2]; // 0.8
				b.sy = sigmas_[3]; // 0.8
				
				gauss_aux.push_back(b);
					
			} else {  //Obtain the group candidates
			
				group_candidate cand;
				geometry_msgs::Pose2D p1;
				p1.x = people_aux.at(i).position.x;
				p1.y = people_aux.at(i).position.y;
				p1.theta = tf::getYaw(people_aux.at(i).orientation);
				cand.position = p1;
				geometry_msgs::Pose2D p2;
				//Calculate the iteraction points
				p2.x = p1.x + (grouping_distance_/2.0)*cos(p1.theta);
				p2.y = p1.y + (grouping_distance_/2.0)*sin(p1.theta);
				p2.theta = p1.theta;
				cand.interaction_point = p2;
				cand.group_id = -1;
				cand.id = people_aux.at(i).id;
				group_candidates.push_back(cand);
			}
		}
		
		//check distance between the interaction points
		vector<group_candidate> group;
		int group_number = 1;
		for(unsigned int i=0; i<group_candidates.size(); i++)
		{
			if(group_candidates.at(i).group_id == -1) {
				group_candidates.at(i).group_id = group_number;
				group.push_back(group_candidates.at(i));
				
				geometry_msgs::Pose2D c1 = group_candidates.at(i).interaction_point;
				for(unsigned int j=0; j<group_candidates.size(); j++)
				{
					if(j!=i && group_candidates.at(j).group_id == -1) {
						geometry_msgs::Pose2D c2 = group_candidates.at(j).interaction_point;
						float dist = sqrt((c1.x-c2.x)*(c1.x-c2.x) + (c1.y-c2.y)*(c1.y-c2.y));
						if(dist < ((grouping_distance_/2.0)*0.9))
						{
							group_candidates.at(j).group_id = group_number;
							group.push_back(group_candidates.at(j));
						}
					}
				}
				
				//Check the members of the group and create the 
				//corresponding gaussians
				if(group.size() > 0)
				{
					//Only one person --> create the regular gaussian function
					if(group.size() == 1)
					{
						gaussian f;
						f.type = FRONT;
						f.x = group.at(0).position.x;
						f.y = group.at(0).position.y;
						f.th = group.at(0).position.theta;
						f.sx = sigmas_[0]; // 1.2
						f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
						
						if(group.at(0).id == it_id_)
							f.type = FRONT_APPROACH;
						
						gauss_aux.push_back(f);
						
						gaussian b;
						b.type = BACK;
						b.x = group.at(0).position.x;
						b.y = group.at(0).position.y;
						float angle = (group.at(0).position.theta) + M_PI;
						b.th = normalizeAngle(angle, -M_PI, M_PI);
						b.sx = sigmas_[2]; // 0.8
						b.sy = sigmas_[3]; // 0.8
						
						gauss_aux.push_back(b);
						
					} else {
						// Calculate the center of the group
						float x = 0.0;
						float y = 0.0;
						float th = 0.0;
						float max_dist = 0.0;
						for(unsigned int k=0; k<group.size(); k++)
						{
							x = x + group.at(k).interaction_point.x;
							y = y + group.at(k).interaction_point.y;
							th = th + group.at(k).interaction_point.theta;
							
							float xk = group.at(k).position.x;
							float yk = group.at(k).position.y;
							for(unsigned int h=0; h<group.size() && h!=k; h++)
							{
								float xh = group.at(h).position.x;
								float yh = group.at(h).position.y;
								float d = sqrt((xk-xh)*(xk-xh) + (yk-yh)*(yk-yh));
								if(d > max_dist)
									max_dist = d;
							}
						}
						
						bool ap = false;
						for(unsigned int r=0; r<group.size() && (ap==false); r++)
						{
							if(group.at(r).id == it_id_)
								ap = true;
						}
						th = normalizeAngle(th, -M_PI, M_PI);
						th = th/group.size();
						th = th + M_PI;
						th = normalizeAngle(th, -M_PI, M_PI);
						
						x = x/group.size();
						y = y/group.size();
						
						gaussian g;
						if(ap)
							g.type = AROUND_APPROACH;
						else
							g.type = AROUND;
						g.x = x;
						g.y = y;
						g.th = th;
						g.sx = sigmas_[2] + max_dist/2.0; // 0.8
						g.sy = sigmas_[2] + max_dist/2.0;  // 0.8
						gauss_aux.push_back(g);
					}
				}
				group.clear();
				group_number++;
			}
		}
		
		
	} else {
		
		for(unsigned int i=0; i<people_aux.size(); i++)
		{
			if(people_aux.at(i).id == it_id_ && it_remove_gauss_)
			{
				continue;
			}
			
			gaussian f;
			f.type = FRONT;
			f.x = people_aux.at(i).position.x;
			f.y = people_aux.at(i).position.y;
			f.th = tf::getYaw(people_aux.at(i).orientation);
			f.sx = sigmas_[0]; // 1.2
			f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
			
			if(people_aux.at(i).id == it_id_)
			{
				f.type = FRONT_APPROACH;
			}
			
			gauss_aux.push_back(f);
			
			gaussian b;
			b.type = BACK;
			b.x = people_aux.at(i).position.x;
			b.y = people_aux.at(i).position.y;
			float angle = (tf::getYaw(people_aux.at(i).orientation)) + M_PI;
			b.th = normalizeAngle(angle, -M_PI, M_PI);
			b.sx = sigmas_[2]; // 0.8
			b.sy = sigmas_[3]; // 0.8
			
			gauss_aux.push_back(b);
			
			
			if(upo_featureset_ == 1 || upo_featureset_ == 2) {
				
				gaussian r;
				r.type = RIGHT;
				r.x = people_aux.at(i).position.x;
				r.y = people_aux.at(i).position.y;
				float angle = (tf::getYaw(people_aux.at(i).orientation)) - M_PI/2.0;
				r.th = normalizeAngle(angle, -M_PI, M_PI);
				r.sx = sigmas_[4]; // 0.8
				r.sy = sigmas_[5]; // 0.4
				
				gauss_aux.push_back(r);
			}
			
		}
	}
	
	gaussianMutex_.lock();
	gaussians_ = gauss_aux;
	gaussianMutex_.unlock();
	
	visualization_msgs::MarkerArray m;
	vector<visualization_msgs::Marker> markers;
	visualization_msgs::Marker marker;
	for(unsigned int i=0; i<gauss_aux.size(); i++)
	{
		marker.header.frame_id = frame; 
		marker.header.stamp = ros::Time::now();
		marker.ns = "gaussians";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = gauss_aux.at(i).x;
		marker.pose.position.y = gauss_aux.at(i).y;
		marker.pose.position.z = 1.1;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(gauss_aux.at(i).th);
		marker.scale.x = 0.7;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(1.0);
		m.markers.push_back(marker);
	}
	if(gauss_aux.size() > 0)
		pub_gaussian_markers_.publish(m);
	
	//printf("\n\n %u Gaussians calculated!!!!\n", (unsigned int)gauss_aux.size());
}



void features::NavFeatures::update()
{
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	calculateGaussians();
	updateDistTransform();
	
}



float features::NavFeatures::proxemicsFeature(geometry_msgs::PoseStamped* s)  {
	
	vector<gaussian> gauss;
	gaussianMutex_.lock();
	gauss = gaussians_;
	gaussianMutex_.unlock();
	
	if(gauss.size() == 0) {
		//printf("gaussian size = 0. Returning 0.0\n");
		return 0.0;
	}
	
	peopleMutex_.lock();
	string people_frame = people_frame_id_;
	peopleMutex_.unlock();
	
	//We need to transform the sample to the same frame of the people
	geometry_msgs::PoseStamped robot = *s;
	robot = transformPoseTo(robot, people_frame, false); //true
	
	if(robot.header.frame_id.empty()){
		printf("frame id empty. returning 0\n");
		return 0.0;
	}
	float rx = robot.pose.position.x; 
	float ry = robot.pose.position.y;
	
	float prox_cost = 1.0;
	for (vector<gaussian>::iterator it = gauss.begin(); it != gauss.end(); ++it)
	{
		//odom coordinates
		float ph = it->th;
		float px = it->x;
		float py = it->y;
		
		/*
		First, transform the robot location into gaussian location frame: 
									|cos(th)  sin(th)  0|
			Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
									|  0        0      1|
					                     
			x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
			y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
		*/
		float x_r = (rx - px)*cos(ph) + (ry - py)*sin(ph);
		float y_r = (rx - px)*(-sin(ph)) + (ry - py)*cos(ph);
		
		//printf("robot x:%.2f, y:%.2f, sx:%.2f, sy:%.2f\n", x_r, y_r, it->sx, it->sy);
		
		float cost = 0.0;
		
		
		if(it->type == FRONT_APPROACH || it->type == AROUND_APPROACH)
		{
			float angle = atan2(y_r, x_r);
			if(angle > (-approaching_angle_) && angle < approaching_angle_ && x_r > 0.8) {
				prox_cost = 0.0;
				return prox_cost;
			} else 
				cost = gaussian_function(x_r, y_r, it->sx, it->sy);
		}
		if(it->type == AROUND) {
			cost = gaussian_function(x_r, y_r, it->sx, it->sy);
		} else if (it->type == FRONT) {
			if(x_r >= 0.0) {
				cost = gaussian_function(x_r, y_r, it->sx, it->sy);
			}
		} else {  //type BACK
			if(x_r > 0.0) {
				cost = gaussian_function(x_r, y_r, it->sx, it->sy);
			}
		}
	
		//prox_cost = prox_cost * (1.0 + cost);
		if((1.0+cost) > prox_cost)
			prox_cost = 1.0+cost;
		
	}
	
	prox_cost = prox_cost - 1.0;	
	//printf("prox_cost: %.3f\n", prox_cost);
	
	//return (prox_cost<=1.0)?prox_cost:1.0;
	if(prox_cost > 1.0) {
		//printf("prox_cost = %.3f\n", prox_cost);
		prox_cost = 1.0;
	}
	//printf("prox_cost: %.3f\n", prox_cost);
	return prox_cost;
}




vector<float> features::NavFeatures::gaussianFeatures(geometry_msgs::PoseStamped* s)
{
	vector<gaussian> gauss;
	gaussianMutex_.lock();
	gauss = gaussians_;
	gaussianMutex_.unlock();
	
	vector<float> pcosts;
	pcosts.push_back(0.0); //Front
	pcosts.push_back(0.0); //Back
	pcosts.push_back(0.0); //Right side
	
	//Remember not to group people to use this feature set
	if(gauss.size() == 0) {
		return pcosts;
	}
	
	peopleMutex_.lock();
	string people_frame = people_frame_id_;
	peopleMutex_.unlock();
	
	//We need to transform the sample to the same frame of the people
	geometry_msgs::PoseStamped robot = *s;
	robot = transformPoseTo(robot, people_frame, false); //true
	
	if(robot.header.frame_id.empty()){
		printf("frame id empty. returning 0\n");
		return pcosts;
	}
	float rx = robot.pose.position.x; 
	float ry = robot.pose.position.y;
	
	
	for (vector<gaussian>::iterator it = gauss.begin(); it != gauss.end(); ++it)
	{
		//odom coordinates
		float ph = it->th;
		float px = it->x;
		float py = it->y;
		
		/*
		First, transform the robot location into gaussian location frame: 
									|cos(th)  sin(th)  0|
			Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
									|  0        0      1|
					                     
			x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
			y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
		*/
		float x_r = (rx - px)*cos(ph) + (ry - py)*sin(ph);
		float y_r = (rx - px)*(-sin(ph)) + (ry - py)*cos(ph);
		
		//printf("robot x:%.2f, y:%.2f, sx:%.2f, sy:%.2f\n", x_r, y_r, it->sx, it->sy);
		

		if (it->type == FRONT) {
			if(x_r >= 0.0) {
				float cost = gaussian_function(x_r, y_r, it->sx, it->sy);
				if(cost > pcosts[0])
					pcosts[0] = cost;
			}
		} else if(it->type == BACK){  
			if(x_r > 0.0) {
				float cost = gaussian_function(x_r, y_r, it->sx, it->sy);
				if(cost > pcosts[1])
					pcosts[1] = cost;
			}
		} else if(it->type == RIGHT) {
			if(x_r > 0.0) {
				float cost = gaussian_function(x_r, y_r, it->sx, it->sy);
				if(cost > pcosts[2])
					pcosts[2] = cost;
			}
		}
		
	}
	
	return pcosts;
}


/*
float features::NavFeatures::proxemicsFeature(geometry_msgs::PoseStamped* s)  {
	
	callbackMutex_.lock();
	unsigned int size = (unsigned int)people_.size();
	callbackMutex_.unlock();
	
	if(size == 0)
		return 0.0;
	
	//We need to transform the state sample to the same people frame
	geometry_msgs::PoseStamped robot = *s;
	
	callbackMutex_.lock();
	robot = transformPoseTo(robot, people_frame_id_, true);
	callbackMutex_.unlock();
	
	float prox_cost = getProxemicsCost((float)robot.pose.position.x, (float)robot.pose.position.y);
	//return prox_cost/amp_;
	return prox_cost;
}*/


/*float features::NavFeatures::getProxemicsCost(float rx, float ry)  {

	// read the person list (thread safe)
	vector<upo_msgs::PersonPoseUPO> people_aux;
	peopleMutex_.lock();
	people_aux = people_;
	peopleMutex_.unlock();
	
	// Cost 0.0 if no persons detected
	if(people_aux.size() == 0) 
	{
		return 0.0;
  	}
  	

	float prox_cost = 1;
	
  	for (vector<upo_msgs::PersonPoseUPO>::iterator it = people_aux.begin(); it != people_aux.end(); ++it)
  	{
		//odom coordinates
		float ph = tf::getYaw(it->orientation);
		float px = it->position.x;
		float py = it->position.y;
		bool front = true;*/
		/*
		First, transform the robot location into person location frame: 
									|cos(th)  sin(th)  0|
			Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
									|  0        0      1|
					                     
			x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
			y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
		*/
		/*float x_r = (rx - px)*cos(ph) + (ry - py)*sin(ph);
		float y_r = (rx - px)*(-sin(ph)) + (ry - py)*cos(ph);
		if(x_r < 0.0)
			front = false;
		
		float cost = gaussian_function(x_r, y_r, front);
		
		prox_cost = prox_cost * (cost + 1.0);
		
	}
	
	prox_cost = prox_cost - 1.0;	
	
	return (prox_cost<=1.0)?prox_cost:1.0;
    
}*/


float features::NavFeatures::gaussian_function(float x, float y, float sx, float sy) 
{
	float g = (amp_*exp(-( (x*x)/(2*(sx*sx)) + (y*y)/(2*(sy*sy))))) / amp_;
	if(g > 1.0) {
		printf("NavFeatures. gaussian value %.2f > 1.0\n", g);
		g = 1.0;
	}
	return g;
}



float features::NavFeatures::gaussian_function(float x, float y, bool fr) 
{
	float sx, sy;
	//front
	if(fr) {
		sx = sigmas_[0]; // 1.2
		sy = sigmas_[1]; // 1.2/1.5 = 0.8
	} else { //back
		sx = sigmas_[2]; // 0.8
		sy = sigmas_[3]; // 0.8
	}
	return (amp_*exp(-( (x*x)/(2*(sx*sx)) + (y*y)/(2*(sy*sy))))) / amp_;

	//phi in radians
	//positions are in odom coordinates
	//Proxemics:
	// intimate: 15-45 cm
	// personal: 46-120 cm
	// social: 120-360 cm
	// public: >360 cm 
	/*double A = amp_;
	double x = x_rob;
	double y = y_rob;
	double x0 = x_per;		
	double y0 = y_per; 
	double z = 0.0;
	
	// 3sigma = 3.60 --> sigma = 1.20
	// 2sigma = 3.60 --> sigma = 1.28
	double sigmaX = sigmas.at(0); //front 1.20
	//Según el artículo "Companion..." de Kirby --> sigmaX = 1.5*sigmaY;
	double sigmaY = sigmas.at(1);  //aside 1.20/1.50 = 0.8
	
	// front
	if(fr) {
	  double a = pow(cos(phi),2)/2/pow(sigmaX,2) + pow(sin(phi),2)/2/pow(sigmaY,2);
	  double b = - sin(2*phi)/4/pow(sigmaX,2) + sin(2*phi)/4/pow(sigmaY,2); 
	  double c = pow(sin(phi),2)/2/pow(sigmaX,2) + pow(cos(phi),2)/2/pow(sigmaY,2);
	  z = A*exp(-(a*pow((x-x0),2) + 2*b*(x-x0)*(y-y0) + c*pow((y-y0),2)));
	  
	} else { //back
	  //take the opposite orientation
	  double o_phi = 0.0;
	  if(phi > 0)
	    o_phi = phi - M_PI;
	  else
	    o_phi = phi + M_PI;
	    
	  sigmaX = sigmas.at(2);
	  sigmaY = sigmas.at(3); 
	  double a = pow(cos(o_phi),2)/2/pow(sigmaX,2) + pow(sin(o_phi),2)/2/pow(sigmaY,2);
	  double b = - sin(2*o_phi)/4/pow(sigmaX,2) + sin(2*o_phi)/4/pow(sigmaY,2); 
	  double c = pow(sin(o_phi),2)/2/pow(sigmaX,2) + pow(cos(o_phi),2)/2/pow(sigmaY,2);
	  z = A*exp(-(a*pow((x-x0),2) + 2*b*(x-x0)*(y-y0) + c*pow((y-y0),2)));
	}
	//ROS_INFO("GC=%.3f, xp=%.1f, yp=%.1f, h=%.1f, xr=%.1f, yr=%.1f, sx=%.1f sy=%.1f",z,x0,y0,phi,x,y,sigmaX,sigmaY);
	return z;*/
}






geometry_msgs::PoseStamped features::NavFeatures::transformPoseTo(geometry_msgs::PoseStamped pose_in, string frame_out, bool usetime)
{
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
		
	geometry_msgs::PoseStamped pose_out;
	
	geometry_msgs::Quaternion q = in.pose.orientation;
	if(!isQuaternionValid(q))
	{
		ROS_WARN("NavFeatures. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	try {
		tf_listener_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("NavFeatures. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	return pose_out;
}




bool features::NavFeatures::isQuaternionValid(const geometry_msgs::Quaternion q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
		ROS_ERROR("Quaternion has infs!!!!");
		return false;
    }
    if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
		ROS_ERROR("Quaternion has nans !!!");
		return false;
	}
	
	if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
		ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
		return false;
	}

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding.");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
}


float features::NavFeatures::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}








