// For an explanation of headers, see SFMExample.cpp
#include "BAExpressions.h"

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

/* ************************************************************************* */
int main (int argc, char* argv[]) 
{
	std::cout << "performing BA" << std::endl;

  	// Find default file, but if an argument is given, try loading a file
  	//string filename = "/home/jrebello/projects/gtsam/examples/Data/dubrovnik-3-7-pre.txt";
  
  	string noisy_filename = "/home/jrebello/catkin_ws/src/acdcc_slam/data/marker_noisy_bal.txt";
  	string true_filename = "/home/jrebello/catkin_ws/src/acdcc_slam/data/marker_true_bal.txt";
  	double sq_size = 0.16;
  	//if (argc>1) filename = string(argv[1]);

  	//boost::shared_ptr<Cal3DS2> K(new Cal3DS2(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230));
	//Cal3DS2 K(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230);
	//Cal3DS2_ cK(K);

	Cal3_S2 K(863.64399, 862.81874, 0, 639.69557, 363.08595);
	Cal3_S2_ cK(K);

  	// Load the SfM data from file
  	SfM_data noisydata, truedata;
  	readBAL(noisy_filename, noisydata);
  	readBAL(true_filename, truedata);

  	cout << boost::format("read %1% tracks on %2% cameras\n") % noisydata.all_points.size() % noisydata.all_poses.size();

  	// Create a factor graph
  	ExpressionFactorGraph graph;
	
  	// We share *one* noiseModel between all projection factors
  	noiseModel::Isotropic::shared_ptr pixelNoise = noiseModel::Isotropic::Sigma(2, 1); // one pixel in u and v
	noiseModel::Diagonal::shared_ptr priorPoseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.00001), Vector3::Constant(0.000003)).finished());
  	noiseModel::Diagonal::shared_ptr priorPointNoise = noiseModel::Isotropic::Sigma(3, 0.00001);
	noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  	noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  	noiseModel::Isotropic::shared_ptr markerNoise = noiseModel::Isotropic::Sigma(2, 0.001);
  	
	graph.addExpressionFactor(Pose3_('x',0), noisydata.all_poses[0], priorPoseNoise); // add directly to graph
  	graph.addExpressionFactor(Point3_('p', 0), noisydata.all_points[0], priorPointNoise); // add directly to graph

	printMarkerInfo(noisydata);
	
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
			//Point2_ prediction = uncalibrate( cK, project( transformTo( Pose3_('x',camera_idx), Point3_('p',point_idx) ) ) );
			graph.addExpressionFactor(prediction, uv, pixelNoise);
			//total_error += (Point2_(uv)-prediction);
			//std::cout << uv.x << " " << uv.y << std::endl;
	    }
	}
	//graph.print("Factor Graph:\n");
	/*
	for(auto it = noisydata.all_marker_ids.begin(); it != noisydata.all_marker_ids.end(); ++it) {
  		vector<int> corners = it->second;
  		for(size_t c=0; c<4; c++){
  			//graph.emplace_shared<ptBetweenFactor>(Symbol('l',corners[c%4]), Symbol('l', corners[c%4]), sq_size, markerNoise);
  		}
	}
  	//std::cout << "Total pixel error: " << total_error << std::endl;
	*/
  	// ********************************* Load Initial Estimates *****************************************************
	
    Values initial;
  	///*
	for(size_t po=0; po<noisydata.all_poses.size(); po++)
  	{
  		Pose3 temp_pose = noisydata.all_poses[po];
  		//Rot3 r = temp_pose.rotation();
  		//Point3 t = temp_pose.translation();
  		//Pose3 pose_new(r,t); 
    	initial.insert(Symbol('x',po), temp_pose);
  	}
  	for(size_t pt=0; pt<noisydata.number_tracks(); pt++)
  	{
    	initial.insert<Point3>(Symbol('p',pt), noisydata.pt_tracks[pt].loc);
  	}
  	//graph.print("Factor Graph:\n");
  	initial.print("Initial estimates:");
	
  	// *********************************** Optimization ******************************************************
  	Values result;
  	try 
  	{
    	LevenbergMarquardtParams params;
    	params.setVerbosity("DEBUG");
    	LevenbergMarquardtOptimizer lm(graph, initial, params);
    	result = lm.optimize();
  	} 
  	catch (exception& e) 
  	{
    	std::cout << e.what() << std::endl;
  	}

  	//cout << "final error: " << graph.error(result) << endl;
	//*/
  	result.print("Final results:\n");
  	cout << "initial error = " << graph.error(initial) << endl;
  	cout << "final error = " << graph.error(result) << endl;

	
  	return 0;
}
/* ************************************************************************* */

