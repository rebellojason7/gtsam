#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Cal3DS2_Base.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <string>

using namespace std;
using namespace gtsam;

// Define Pinhole camera model with distortion
typedef gtsam::PinholePose<Cal3DS2> PinholeCamera_Cal3DS2;

//typedef Expression<Cal3DS2> Cal3DS2_;

// Camera ID with pixel measurement
typedef std::pair<size_t, Point2> SfM_Measurement;

/// Define the structure for the 3D points
struct SfM_Track {
  
  SfM_Track():loc(0,0,0) {}
  Point3 loc; // 3D position of the point in origin (1st Keyframe)
  bool marker_id; // Id of marker else -1 if natural feature
  bool corner_num; // 1 - top left, 2 - top right, 3 - bottom right, 4 - bottom left
  float r, g, b; // RGB color of the 3D point
  std::vector<SfM_Measurement> measurements; ///< The 2D image projections (id,(u,v))
  
  size_t number_measurements() const { // Number of measurements for a point
    return measurements.size();
  }
};

// Define the structure for SfM data
struct SfM_data {
  
  std::vector<SfM_Track> pt_tracks; ///< Sparse set of points
  std::vector<Pose3> all_poses;
  std::vector<Point3> all_points;
  std::map<string, std::vector<int> > all_marker_ids;
  
  size_t number_tracks() const { // Number of 3D points
    return pt_tracks.size();
  } 
};

// Read BAL file format
bool readBAL(const string& filename, SfM_data &data) {
  // Load the data file
  ifstream is(filename.c_str(), ifstream::in);
  if (!is) {
    cout << "Error in readBAL: can not find the file!!" << endl;
    return false;
  }

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints, nrMarkers, nrObservations;
  is >> nrPoses >> nrPoints >> nrMarkers >> nrObservations;
  //std::cout << nrPoses << " " << nrPoints << " " << nrMarkers << " " << nrObservations << std::endl;

  data.pt_tracks.resize(nrPoints);

  // Get the information for the observations
  for (size_t k = 0; k < nrObservations; k++) {
    int cam_idx=0, pt_idx=0, m_id=0, m_corner=0;
    float pixel_u = 0.0, pixel_v=0.0;
    is >> cam_idx >> pt_idx >> pixel_u >> pixel_v >> m_id >> m_corner;

    // For the point, store the corresponding keyframe and pixel measurements.
    data.pt_tracks[pt_idx].measurements.emplace_back(cam_idx, gtsam::Point2(pixel_u, pixel_v));

    if(m_id!=-1){ // This is a tag corner
    	if (data.all_marker_ids.find(to_string(m_id)) != data.all_marker_ids.end()){
    		data.all_marker_ids[to_string(m_id)][m_corner-1] = pt_idx;
    	}
    	else{
    		vector<int> default_corners = {-1, -1, -1, -1};
    		default_corners[m_corner-1] = pt_idx; 
    		data.all_marker_ids[to_string(m_id)] = default_corners;
    	}
    }
  }

  // Get the information for the camera poses
  for (size_t k = 0; k < nrPoses; k++) {
    gtsam::Matrix4 m_pose;
    for(size_t m=0; m<4; m++){
      // Need to write this, read 4x4 matrix for pose
      float i, j, u, v;
      is >> i >> j >> u >> v;
      m_pose(m,0) = i;
      m_pose(m,1) = j;
      m_pose(m,2) = u;
      m_pose(m,3) = v;
      //std::cout << i << " " << j << " " << u << " " << v << std::endl;
    }
    gtsam::Pose3 pose(m_pose); 
    //std::cout << pose << std::endl;
    //data.cameras.emplace_back(pose,K);
    data.all_poses.push_back(pose);
  }

  // Get the information for the 3D points
  for (size_t j = 0; j < nrPoints; j++) {
    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    SfM_Track& track = data.pt_tracks[j];
    track.loc = gtsam::Point3(x, y, z);
    track.r = 0.4f;
    track.g = 0.4f;
    track.b = 0.4f;
    data.all_points.push_back(track.loc);
  }

  is.close();
  return true;
}

void printMarkerInfo(const SfM_data &data){
	for(auto it = data.all_marker_ids.begin(); it != data.all_marker_ids.end(); ++it) {
  		cout << "Marker ID: " << it->first << " Corners: " << it->second[0] << " " << it->second[1] << " " << it->second[2] << " " << it->second[3] << std::endl;
	}
}
