// For an explanation of headers, see SFMExample.cpp
#include "BA.h"

class ptBetweenFactor: public NoiseModelFactor2<Point3, Point3>{
	double marker_size;

public:
	typedef boost::shared_ptr<ptBetweenFactor> shared_ptr;
	ptBetweenFactor(Key i, Key j, double m_size, const SharedNoiseModel& model):
		NoiseModelFactor2<Point3, Point3>(model, i, j), marker_size(m_size) {}

	virtual ~ptBetweenFactor() {}

	double evaluateError(const Point3 &p1, const Point3 &q, OptionalJacobian<1, 3> H1, OptionalJacobian<1, 3> H2) const
	{
  		double d = (q - p1).norm();
  		if (H1) 
  		{
    		*H1 << p1.x() - q.x(), p1.y() - q.y(), p1.z() - q.z();
    		*H1 = *H1 *(1. / d);
  		}
  		if (H2) 
  		{
    		*H2 << -p1.x() + q.x(), -p1.y() + q.y(), -p1.z() + q.z();
    		*H2 = *H2 *(1. / d);
  		}
  		return d;
	}

	double measured() const
	{
		return 0.16;
	}

	size_t size() const
	{
		return 2;
	}
	
	/*
	virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new ptBetweenFactor(*this))); }
  */      
};

/* ************************************************************************* */
int main (int argc, char* argv[]) 
{
	std::cout << "performing BA" << std::endl;

  	// Find default file, but if an argument is given, try loading a file
  	//string filename = "/home/jrebello/projects/gtsam/examples/Data/dubrovnik-3-7-pre.txt";
  
  	string noisy_filename = "/home/jrebello/catkin_ws/src/acdcc_slam/data/marker_true_bal.txt";
  	string true_filename = "/home/jrebello/catkin_ws/src/acdcc_slam/data/marker_true_bal.txt";
  	double sq_size = 0.16;
  	//if (argc>1) filename = string(argv[1]);

  	boost::shared_ptr<Cal3DS2> K(new Cal3DS2(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230));

  	// Load the SfM data from file
  	SfM_data noisydata, truedata;
  	readBAL(noisy_filename, noisydata);
  	readBAL(true_filename, truedata);

  	cout << boost::format("read %1% tracks on %2% cameras\n") % noisydata.all_points.size() % noisydata.all_poses.size();

  	// Create a factor graph
  	NonlinearFactorGraph graph;

  	// We share *one* noiseModel between all projection factors
  	noiseModel::Isotropic::shared_ptr pixelNoise = noiseModel::Isotropic::Sigma(2, 1); // one pixel in u and v
  	noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  	noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  	noiseModel::Isotropic::shared_ptr markerNoise = noiseModel::Isotropic::Sigma(2, 0.001);
  	graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), noisydata.all_poses[0], poseNoise); // add directly to graph
  	//graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', 0), all_points[0], pointNoise); // add directly to graph

  	//initialEstimate.print("Initial Estimates:\n");

  	// ********************************** Add Factors to graph *******************************************************
  	double total_error = 0.0;
  	std::cout << std::endl;
  	std::cout << "==============================" << std::endl;
  	for(size_t t=0; t<noisydata.number_tracks(); t++)
  	{
    	size_t point_idx = t;
    	for(size_t m=0; m<noisydata.pt_tracks[t].measurements.size(); m++)
    	{
      		size_t camera_idx = noisydata.pt_tracks[t].measurements[m].first; 
      		Point2 uv = noisydata.pt_tracks[t].measurements[m].second;
      		graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3DS2> >(uv, pixelNoise, Symbol('x', camera_idx), Symbol('l', point_idx), K);
      		//std::cout << camera_idx << " " << point_idx << std::endl;
	      	PinholeCamera_Cal3DS2 cam(noisydata.all_poses[camera_idx], *K);
	      	//std::cout << "Camera pose: " << cam.pose() << std::endl;
	      	//std::cout << "Point Location: " << all_points[point_idx] << std::endl;
	      	Point2 meas = cam.project(noisydata.all_points[point_idx]);
	      	//std::cout << "true pixels:" << uv << " , projected pixels:" << meas << std::endl;
	      	//std::cout << (uv - meas).norm() << std::endl;
	      	total_error += (uv-meas).norm();
	      	//std::cout << "==============================" << std::endl;
	      	//std::cout << std::endl;
	    }
	}
	for(auto it = noisydata.all_marker_ids.begin(); it != noisydata.all_marker_ids.end(); ++it) {
  		vector<int> corners = it->second;
  		for(size_t c=0; c<4; c++){
  			//graph.emplace_shared<ptBetweenFactor>(Symbol('l',corners[c%4]), Symbol('l', corners[c%4]), sq_size, markerNoise);
  		}
	}
	graph.print("Factor Graph:\n");
  	//std::cout << "Total pixel error: " << total_error << std::endl;

  	// ********************************* Load Initial Estimates *****************************************************
    Values initialEstimate;
  	for(size_t po=0; po<noisydata.all_poses.size(); po++)
  	{
  		Pose3 temp_pose = noisydata.all_poses[po];
  		Rot3 r = temp_pose.rotation();
  		Point3 t = temp_pose.translation();
  		Pose3 pose_new(r,t); 
    	initialEstimate.insert(Symbol('x',po), pose_new);
  	}
  	for(size_t pt=0; pt<noisydata.number_tracks(); pt++)
  	{
    	initialEstimate.insert<Point3>(Symbol('l',pt), noisydata.pt_tracks[pt].loc);
  	}
  	//graph.print("Factor Graph:\n");
  	//initialEstimate.print("Initial estimates:");
  	//std::cout <<  std::endl;
  
  	// *********************************** Optimization ******************************************************
  	Values result;
  	try 
  	{
    	LevenbergMarquardtParams params;
    	params.setVerbosity("DEBUG");
    	LevenbergMarquardtOptimizer lm(graph, initialEstimate, params);
    	result = lm.optimize();
  	} 
  	catch (exception& e) 
  	{
    	std::cout << e.what() << std::endl;
  	}

  	cout << "final error: " << graph.error(result) << endl;
  	//result.print("Final results:\n");
  	//cout << "initial error = " << graph.error(initialEstimate) << endl;
  	//cout << "final error = " << graph.error(result) << endl;
  	return 0;
}
/* ************************************************************************* */

