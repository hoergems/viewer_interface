#ifndef VIEWER_INTERFACE_HPP_
#define VIEWER_INTERFACE_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include "viewer.hpp"
#include "urdf_loader.hpp"
#include <Eigen/Dense>

namespace shared {

class ViewerInterface {
public:
	ViewerInterface();
	
	~ViewerInterface() {
		if (viewer_setup_) { 
			OpenRAVE::RaveDestroy(); 
		}
	}
	
	bool setupViewer(std::string model_file,
                     std::string environment_file);
	
	bool addObstacle(std::string &name,
			         std::vector<double> &dims);
	
	bool removeObstacle(std::string &name);
	
	bool removeObstacles();
	
	void updateRobotValues(const std::vector<double> &current_joint_values,
		   	   		       const std::vector<double> &current_joint_velocities,	
						   const std::vector<std::vector<double>> &particle_joint_values,
						   const std::vector<std::vector<double>> &particle_colors,
		   	   			   OpenRAVE::RobotBasePtr robot);
	
	void addSensor(std::string &sensor_file);
	
	void setSensorTransform(Eigen::MatrixXd &transform);
	
	void sensor_loop();
	
	/**
	 * Add particles that are independent of viewer updates
	 */
	void addPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
			                   const std::vector<std::vector<double>> &particle_colors);
	
	/**
	 * Remove the permanently added particles
	 */
	void removePermanentParticles();
	
    /**
     * Set the size of the viewer attached to the environment
     */
    void setViewerSize(int x, int y);

    /**
     * Set the background color of the viewer
     */
    void setBackgroundColor(double &r, double &g, double &b);
    
    /**
     * Set the camera transform
     */
    void setCameraTransform(std::vector<double> &rot, std::vector<double> &trans);
    
    /**
     * Maximum number of particles to plot
     */
    void setParticlePlotLimit(unsigned int particle_plot_limit);
    
    void setObstacleColor(std::string &obstacle_name, 
       		              std::vector<double> &diffuse_color,
       		              std::vector<double> &ambient_color);
    
    bool addBox(std::string &name, std::vector<double> &dims);

private:
    bool viewer_setup_;
    
    bool robot_added_;
    
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::KinBodyPtr robot_;
    
    std::string model_file_;
    
    OpenRAVE::RobotBasePtr getRobot();
    
    std::shared_ptr<shared::URDFLoader> urdf_loader_;
    
    std::shared_ptr<shared::RaveViewer> viewer_;
    
    unsigned int particle_plot_limit_;
    
    //boost::thread sensor_thread_;
    
    OpenRAVE::SensorBasePtr sensor_;
    
};


}

#endif