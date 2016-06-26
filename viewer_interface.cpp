#include "viewer_interface.hpp"
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <unistd.h>


using std::cout;
using std::endl;

namespace shared {

ViewerInterface::ViewerInterface ():
	viewer_setup_(false),
	robot_added_(false),
	env_(),
	robot_(nullptr),
	urdf_loader_(),
	viewer_(new shared::RaveViewer()),
	particle_plot_limit_(50),
	sensor_(nullptr){
}

void ViewerInterface::sensor_loop() {	
	sensor_ = env_->GetSensor("mysensor_blubb");
	if (sensor_) {
	    sensor_->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
	    sensor_->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
	    OpenRAVE::SensorBase::SensorDataPtr sensor_data = sensor_->CreateSensorData(OpenRAVE::SensorBase::SensorType::ST_Laser);
	    boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> laser_data = 
	    		boost::static_pointer_cast<OpenRAVE::SensorBase::LaserSensorData>(sensor_data);
	    while (true) {	
	    	sensor_->GetSensorData(sensor_data);
	    	for (size_t i = 0; i < laser_data->intensity.size(); i++) {
	    		if (laser_data->intensity[i] > 0) {
	    			cout << "got one" << endl;
	    			cout << "(" << i << ", " << laser_data->intensity[i] << ")" << endl;
	    			break;
	    		}
	    	}
	    	cout << "looking" << endl;
	    	usleep(2.0 * 1e6);
	    }
	}
}

void ViewerInterface::setSensorTransform(Eigen::MatrixXd &transform) {	
	Eigen::MatrixXd rot1(4, 4);
	double angle = M_PI;
	rot1 << cos(angle), 0.0, sin(angle), 0.01,
			0.0, 1.0, 0.0, 0.0,
			-sin(angle), 0.0, cos(angle), 0.0,
			0.0, 0.0, 0.0, 1.0;
    transform = transform * rot1;
	Eigen::Matrix3d mat;
	mat << transform(0, 0), transform(0, 1), transform(0, 2),
		   transform(1, 0), transform(1, 1), transform(1, 2),
		   transform(2, 0), transform(2, 1), transform(2, 2);
	Eigen::Quaternion<double> quat(mat);
	OpenRAVE::geometry::RaveVector<double> rot(quat.x(), quat.y(), quat.z(), quat.w());
	OpenRAVE::geometry::RaveVector<double> trans(transform(0, 3), 
			                                     transform(1, 3),
												 transform(2, 3));
	const OpenRAVE::Transform sensor_trans(rot, trans);
	sensor_->SetTransform(sensor_trans);
}

void ViewerInterface::addSensor(std::string &sensor_file) {	
	//test();
	OpenRAVE::RaveInitialize(true);	
	OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Debug);
	env_ = OpenRAVE::RaveCreateEnvironment();
	env_->Load("/home/hoe01h/LQG_Newt/environment/env_3dof.xml");
	//OpenRAVE::ViewerBasePtr viewer;
	//viewer = OpenRAVE::RaveCreateViewer(env_, "qtcoin");
	//env_->Add(viewer);	
	boost::thread thopenrave(boost::bind(&ViewerInterface::sensor_loop,this));	
	//viewer->main(true);
}

bool ViewerInterface::setupViewer(std::string model_file,
                                  std::string environment_file) {
	if (viewer_setup_) {
		return false;
    }
	
	model_file_ = model_file;	
	OpenRAVE::RaveInitialize(true);	
	OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Debug);
	env_ = OpenRAVE::RaveCreateEnvironment();
	env_->SetPhysicsEngine(nullptr);
	//env_->SetCollisionChecker(nullptr);
	//env_->StopSimulation();
	cout << "loading " << environment_file << endl;
	env_->Load(environment_file);	
	cout << "loaded environment" << endl;	
	robot_ = urdf_loader_->load(model_file, env_);		
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	//env_->StopSimulation();	
	for (auto &k: bodies) {
		cout << k->GetName() << endl;
	}
	
	/**OpenRAVE::RobotBasePtr robot = getRobot();	
	const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());	
	for (size_t i = 0; i < links.size(); i++) {			
		if (links[i]->GetName() == "world") {
			links[i]->SetStatic(true);
		}
		else if (links[i]->GetName() == "end_effector") {
			links[i]->Enable(false);
		}
	}
	
	boost::thread sensor_thread(boost::bind(&ViewerInterface::sensor_loop, this));*/	
	shared::RaveViewer viewer;
	viewer_->testView(env_);
	cout << "Initialized viewer" << endl;
	viewer_setup_ = true;
	return true;
	
}

bool ViewerInterface::addBox(std::string &name,
		                     std::vector<double> &dims) {
	if (env_) {
		// We remove the box with the same name first
		std::vector<OpenRAVE::KinBodyPtr> bodies;
	    env_->GetBodies(bodies);
		for (auto &body: bodies) {
			if (body->GetName().find(name) != std::string::npos) {			
				env_->Remove(body);
			}
		}
		
		OpenRAVE::KinBodyPtr kin_body = OpenRAVE::RaveCreateKinBody(env_);		
		OpenRAVE::Vector trans(dims[0], dims[1], dims[2]);
		OpenRAVE::Vector extents(dims[3], dims[4], dims[5]);
		
		Eigen::VectorXd x(3);
		Eigen::VectorXd y(3);		
		x << 1.0, 0.0, 0.0;
		y << 0.0, 1.0, 0.0;		
		
		Eigen::MatrixXd rot(3, 3);
		
		rot << cos(dims[6]), -sin(dims[6]), 0.0,
			   sin(dims[6]), cos(dims[6]), 0.0,
			   0.0, 0.0, 1.0;
		
		Eigen::VectorXd x_rot = rot * x;
		Eigen::VectorXd y_rot = rot * y;
		
		OpenRAVE::OBB obb;
		obb.right = OpenRAVE::Vector(y_rot[0], y_rot[1], y_rot[2]);
		obb.up = OpenRAVE::Vector(0.0, 0.0, 1.0);
		obb.dir = OpenRAVE::Vector(x_rot[0], x_rot[1], x_rot[2]);
		obb.pos = trans;
		obb.extents = extents;
		std::vector<OpenRAVE::OBB> obb_vec({obb});
		const std::vector<OpenRAVE::OBB> const_r = obb_vec;
		kin_body->SetName(name);
		kin_body->InitFromBoxes(obb_vec);
		kin_body->Enable(true);
		env_->Add(kin_body, true);
		return true;
	}
	
	return false;
}

bool ViewerInterface::addObstacle(std::string &name,
			                      std::vector<double> &dims) {
	if (env_) {
		OpenRAVE::KinBodyPtr kin_body = OpenRAVE::RaveCreateKinBody(env_);
		std::vector<OpenRAVE::AABB> aabb_vec;
		cout << "Add box: " << dims[0] << ", " << dims[1] << ", " << dims[2] << ", " << dims[3] << ", " << dims[4] << ", " << dims[5] << endl;
		OpenRAVE::Vector trans(dims[0], dims[1], dims[2]);
		OpenRAVE::Vector extents(dims[3], dims[4], dims[5]);
		OpenRAVE::AABB aabb(trans, extents);
		aabb_vec.push_back(aabb);
		const std::vector<OpenRAVE::AABB> const_rave_boxes = aabb_vec;
		kin_body->SetName(name);
		kin_body->InitFromBoxes(const_rave_boxes, true);
		kin_body->Enable(false);
		env_->Add(kin_body, true);
		return true;
	}
}

bool ViewerInterface::removeObstacle(std::string &name) {
	if (env_) {
		std::vector<OpenRAVE::KinBodyPtr> bodies;
		env_->GetBodies(bodies);
		for (auto &body: bodies) {
			if (body->GetName().find(name) != std::string::npos) {			
				env_->Remove(body);
			}
		}
	}
}

bool ViewerInterface::removeObstacles() {
	if (env_) {
		std::vector<OpenRAVE::KinBodyPtr> bodies;		
		env_->GetBodies(bodies);		
		std::string particle_string = "obst";
				
		// Remove the particle bodies from the scene	
		for (auto &body: bodies) {		
			if (body->GetName().find(particle_string) != std::string::npos) {			
				env_->Remove(body);
			}		
		}
	}
}

void ViewerInterface::setViewerSize(int x, int y) {
	viewer_->setViewerSize(x, y);
}

void ViewerInterface::setBackgroundColor(double &r, double &g, double &b) {
	viewer_->setBackgroundColor(r, g, b);
}

void ViewerInterface::setCameraTransform(std::vector<double> &rot, std::vector<double> &trans) {
	viewer_->setCameraTransform(rot, trans);
}

void ViewerInterface::getCameraImage(std::vector<uint8_t> &image, int width, int height) {
	if (env_) {
		OpenRAVE::ViewerBasePtr viewer = env_->GetViewer("viewer");
		const OpenRAVE::RaveTransform<float> cameraTransform = viewer->GetCameraTransform();
		OpenRAVE::geometry::RaveCameraIntrinsics<float> cameraIntrinsics = viewer->GetCameraIntrinsics();
		const OpenRAVE::geometry::RaveCameraIntrinsics<double> intrinsics(cameraIntrinsics.fx,
				                                                          cameraIntrinsics.fy,
				                                                          cameraIntrinsics.cx,
				                                                          cameraIntrinsics.cy);
		viewer->GetCameraImage(image, width, height, cameraTransform, intrinsics);		
	}
}

void ViewerInterface::removePermanentParticles() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    // Remove the particle bodies from the scene
    std::string particle_string = "permanent_";
    for (auto &body: bodies) {		
    	if (body->GetName().find(particle_string) != std::string::npos) {			
    		env_->Remove(body);
    	}		
    }
}

void ViewerInterface::setObstacleColor(std::string &obstacle_name, 
 		                               std::vector<double> &diffuse_color,
 		                               std::vector<double> &ambient_color) {
 	std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {    	
    	const std::vector<OpenRAVE::KinBody::LinkPtr> links(body->GetLinks());
    	for (auto &link: links) {
    		const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries(link->GetGeometries());
    		for (auto &geom: geometries) {
    			if (body->GetName().find(obstacle_name) != std::string::npos) {
    				const OpenRAVE::RaveVector<float> d_color(diffuse_color[0],
    						                                  diffuse_color[1],
    						                                  diffuse_color[2],
    						                                  diffuse_color[3]);
    				const OpenRAVE::RaveVector<float> a_color(ambient_color[0],
    				    						              ambient_color[1],
    				    						              ambient_color[2],
    				    						              ambient_color[3]);
    				geom->SetDiffuseColor(d_color);
    				geom->SetAmbientColor(a_color);
    						                                        		
    			}
     		}		
     	}
     }
 }

void ViewerInterface::addPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
		                                    const std::vector<std::vector<double>> &particle_colors) {	
	double num_plot = particle_joint_values.size();
	for (size_t i = 0; i < num_plot; i++) {
	    OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(model_file_, env_);
		std::string name = "permanent_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
	    env_->Add(robot_ptr, true);
	    std::vector<OpenRAVE::dReal> joint_vals;
	    for (auto &k: particle_joint_values[i]) {
	        joint_vals.push_back(k);
	    }
	        
	    joint_vals.push_back(0);
	        
	    robot_ptr->SetDOFValues(joint_vals);
	        
	    const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
	    for (auto &link: links) {
	        const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
	        for (auto &geometry: link_geometries) {
	            if (geometry->IsVisible()) {
	        	    OpenRAVE::Vector color(particle_colors[i][0], 
	        					           particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
	        		geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
	        		geometry->SetTransparency(0.75);
	        	}
	        }
	    }
	}
	
}

void ViewerInterface::setParticlePlotLimit(unsigned int particle_plot_limit) {	
	particle_plot_limit_ = particle_plot_limit;
}

void ViewerInterface::updateRobotValues(const std::vector<double> &current_joint_values,
		                                const std::vector<double> &current_joint_velocities,
										const std::vector<std::vector<double>> &particle_joint_values,
										const std::vector<std::vector<double>> &particle_colors,
								        OpenRAVE::RobotBasePtr robot=nullptr) {	
	if (!robot_added_) {		
		env_->Add(robot_, true);
		robot_added_ = true;
		cout << "loaded robot" << endl;
	}
	OpenRAVE::RobotBasePtr robot_to_use(nullptr);
	
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	std::string particle_string = "particle";
		
	// Remove the particle bodies from the scene	
	for (auto &body: bodies) {		
		if (body->GetName().find(particle_string) != std::string::npos) {			
			env_->Remove(body);
		}		
	}
			
	if (robot == nullptr) {
		robot_to_use = getRobot();
	}
	else {
		robot_to_use = robot;
	}
	if (robot_to_use == nullptr) {
		cout << "Propagator: Error: Environment or robot has not been initialized or passed as argument. Can't propagate the state" << endl;
		return;	
	}
	
	std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_to_use->GetLinks();
	std::vector<OpenRAVE::KinBody::JointPtr> joints = robot_to_use->GetJoints();	
	std::vector<OpenRAVE::KinBody::LinkInfo> link_infos;
	std::vector<OpenRAVE::KinBody::JointInfo> joint_infos;	
	for (auto &link: links) {
		link_infos.push_back(link->GetInfo());
	}
	
	for (auto &joint: joints) {
		joint_infos.push_back(joint->GetInfo());
	}
	
	std::vector<OpenRAVE::dReal> newJointValues;
	for (size_t i = 0; i < current_joint_values.size(); i++) {		
		if (current_joint_values[i] < -M_PI) {
			newJointValues.push_back(2.0 * M_PI + current_joint_values[i]);
		}
		else if (current_joint_values[i] > M_PI) {
			newJointValues.push_back(-2.0 * M_PI + current_joint_values[i]);
		}
		else {
			newJointValues.push_back(current_joint_values[i]);
		}
	}	
		
	newJointValues.push_back(0);	
	robot_to_use->SetDOFValues(newJointValues);
	size_t num_plot = particle_plot_limit_;	
	if (particle_joint_values.size() < num_plot) {
		num_plot = particle_joint_values.size();
	}
	
	// Here we plot non-permanent particles
	for (size_t i = 0; i < num_plot; i++) {
		OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(model_file_, env_);
		std::string name = "particle_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
        env_->Add(robot_ptr, true);
        std::vector<OpenRAVE::dReal> joint_vals;
        for (auto &k: particle_joint_values[i]) {
        	joint_vals.push_back(k);
        }
        
        joint_vals.push_back(0);        
        robot_ptr->SetDOFValues(joint_vals);        
        const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
        for (auto &link: links) {
        	const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
        	for (auto &geometry: link_geometries) {
        		if (geometry->IsVisible()) {
        			OpenRAVE::Vector color(particle_colors[i][0], 
        					               particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
        			geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
        			geometry->SetTransparency(particle_colors[i][3]);
        		}
        	}
        }
	}
}

OpenRAVE::RobotBasePtr ViewerInterface::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {
    	if (body->GetDOF() > 0) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}

}