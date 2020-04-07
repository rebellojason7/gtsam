/*
Author : Jason Rebello
This is the file that is used to solve a final optimization after the map UcoSLAM
spits out.
The path to the file and square size need to be modified.

Note: The intrinsics do not remain fixed. Need to Solve that issue !!!!!!
*/
#include "ACDCCOptimization.h"

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using symbol_shorthand::C;
using symbol_shorthand::P;

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

  	boost::shared_ptr<Cal3DS2> const K(new Cal3DS2(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230));
	//Cal3DS2 K(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230);
	//Cal3DS2_ cK(K);

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
  	noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.5);
  	auto markerNoise = noiseModel::Diagonal::Sigmas(Vector1(0.001));;
  	
	Expression<PinholeCamera_Cal3DS2> camera0_(C(0));
	Pose3_ pose0_(&PinholeCamera_Cal3DS2::getPose, camera0_);
	graph.addExpressionFactor(pose0_, noisydata.all_poses[0], priorPoseNoise); // add directly to graph
  	Point3_ point0_(P(0));
	graph.addExpressionFactor(point0_, noisydata.all_points[0], priorPointNoise); // add directly to graph

	printMarkerInfo(noisydata);
	
  	// ********************************** Add Factors to graph *******************************************************
	std::cout << "Adding Projection Factors" << std::endl;
	size_t j = 0;
  	for(const SfM_Track& track: noisydata.pt_tracks) {
    	// Leaf expression for j^th point
    	Point3_ point_('p', j);
    	for(const SfM_Measurement& m: track.measurements) {
      		size_t i = m.first;
      		Point2 uv = m.second;
      		// Leaf expression for i^th camera
			Expression<PinholeCamera_Cal3DS2> camera_(C(i));
			// Below an expression for the prediction of the measurement:
			Point2_ predict_ = project2<PinholeCamera_Cal3DS2>(camera_, point_);
			// Again, here we use an ExpressionFactor
			graph.addExpressionFactor(predict_, uv, pixelNoise);
    	}
    	j += 1;
  	}

	std::cout << "Adding Point Constraint factors" << std::endl;
	for (std::map<string,std::vector<int>>::iterator it=noisydata.all_marker_ids.begin(); it!=noisydata.all_marker_ids.end(); ++it){
    	std::cout << "Adding marker ID: " << it->first << std::endl;
		std::vector<int> corner_vec = it->second;
		for(int c=0; c<4; c++){
			int idx_pt1=c%4, idx_pt2=(c+1)%4;
			int c1=corner_vec[idx_pt1], c2=corner_vec[idx_pt2];		
			auto h = Double_(&distance3, Point3_(P(c1)), Point3_(P(c2)));
			graph.addExpressionFactor(h, sq_size, markerNoise);
		}
	}
	//graph.print("Factor Graph:\n");

  	// ********************************* Load Initial Estimates *****************************************************
	
	Values initial;
  	size_t i = 0;
  	j = 0;
  	//for(const PinholeCamera_Cal3DS2& camera: noisydata.all_poses)
	for(auto i=0; i<noisydata.all_poses.size(); i++)
	{
		PinholeCamera_Cal3DS2 cam(noisydata.all_poses[i], K);
   		initial.insert(C(i), cam);
	}
  	for(const SfM_Track& track: noisydata.pt_tracks)
    	initial.insert(P(j++), track.loc);
	
	//initial.print("Initial estimates:");
	
	///*
	Values result;
  	try {
    	LevenbergMarquardtParams params;
    	params.setVerbosity("ERROR");
    	LevenbergMarquardtOptimizer lm(graph, initial, params);
    	result = lm.optimize();
  	} catch (exception& e) {
    	cout << e.what();
  	}
	//result.print("Final results:\n");

	std::vector<Point3> opt_p_vec;
	int p_idx = 0;
	for(auto it=result.begin(); it!=result.end(); it++)
	{	
		string s = _defaultKeyFormatter(it->key);
		string letter = s.substr(0,1);
		if(letter.compare("p") == 0){
			Point3 p_temp = result.at<Point3>(it->key);
			//opt_p_vec.push_back(p_temp);
			Vector3 p_diff = p_temp.vector() - truedata.all_points[p_idx].vector();
			std::cout << p_diff.transpose() << std::endl; 
			p_idx++;
		}
	}
	
  	return 0;
}
/* ************************************************************************* */

