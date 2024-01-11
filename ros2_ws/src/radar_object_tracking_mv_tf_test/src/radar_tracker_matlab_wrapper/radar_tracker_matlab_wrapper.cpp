#include <iostream>
#include <ios>
#include <string>
#include <fstream>
#include <map>
#include <unistd.h>
#include <limits.h>
#include <boost/format.hpp>

#include "radar_tracker_matlab_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono> 
using namespace std::chrono;


CRadarTrackerMatlabWrapper::CRadarTrackerMatlabWrapper()
{
  std::cout << "[CRadarTrackerMatlabWrapper::CRadarTrackerMatlabWrapper()] Calling matlab generated init() functions..." << std::endl;

  // Init some matlab generated constants like rtInf...
  radar_tracker_initialize();

  // Our own init: Empty object lists...
  radar_tracker_init(m_obj_list_ast);

#if !TF_SWITCH_ON
  m_meas_list_st.dx_sens_offset = 0;
  m_meas_list_st.dy_sens_offset = 0; 
  m_meas_list_st.ang_sens_offset = 0;
#endif

  std::cout << "[CRadarTrackerMatlabWrapper::CRadarTrackerMatlabWrapper()] Done" << std::endl;

  //! m_t_last_measurement = ros::Time(0);
  m_t_last_measurement = rclcpp::Time(0);


  // Show current directory, so we know where to look for the debug.txt file
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != NULL)
  {
    std::cout << "[CRadarTrackerMatlabWrapper::CRadarTrackerMatlabWrapper()] Current working dir: " << cwd << std::endl;
  }
}


CRadarTrackerMatlabWrapper::~CRadarTrackerMatlabWrapper()
{
  std::cout << "[CRadarTrackerMatlabWrapper::~CRadarTrackerMatlabWrapper()] Goodbye" << std::endl;
}

// Executes one cycle of the tracker
void CRadarTrackerMatlabWrapper::runTrackingCycle()
{

  //auto start = high_resolution_clock::now();

  // Call main function
  radar_tracker(m_obj_list_ast, &m_meas_list_st, m_current_dt.seconds());

  auto stop = high_resolution_clock::now(); 
  // Obj_type_classification(m_obj_list_ast);

  //auto duration = duration_cast<milliseconds>(stop - start);
  //std::cout << "runtime: " << duration.count() << "ms \n" << std::endl;

}

// converts the (internal) 2D cov to a cov in 3D world and returns it
cov3d CRadarTrackerMatlabWrapper::getObj3DCovariance(std::size_t idx) const
{
  cov3d current_state_3d_cov = {};

  // 2d state vector: [px, vx, ax, py, vy, ay]
  // 3d state vecotr: [px, py, pz, vx, vy, vz, ax, ay, az]
  std::map<int,int> state_vec_ind_2d_2_3d;
  state_vec_ind_2d_2_3d[0] = 0; //px
  state_vec_ind_2d_2_3d[3] = 1; //py
  state_vec_ind_2d_2_3d[1] = 3; //vx
  state_vec_ind_2d_2_3d[4] = 4; //vy
  state_vec_ind_2d_2_3d[2] = 6; //ax
  state_vec_ind_2d_2_3d[5] = 7; //ay


  // iterate over cols and rows of the 2d state covariance
  for( int cov_2d_row_ind = 0; cov_2d_row_ind < 6; cov_2d_row_ind++)
  {
    // get indices in 2d and 3d state vector for row
    int cov_3d_row_ind = state_vec_ind_2d_2_3d[cov_2d_row_ind];
    for( int cov_2d_col_ind = 0; cov_2d_col_ind < 6; cov_2d_col_ind++)
    {
      // get indices in 2d and 3d state vector for col
      int cov_3d_col_ind = state_vec_ind_2d_2_3d[cov_2d_col_ind];

      // calculate index in the matrix P (covariance of x)
      // 6 is the length of x, 9 the lenght of x3d
      int ind_2d_cov = 6 * cov_2d_col_ind + cov_2d_row_ind;
      int ind_3d_cov = 9 * cov_3d_col_ind + cov_3d_row_ind;

      current_state_3d_cov[ind_3d_cov] = m_obj_list_ast[idx].P[ind_2d_cov];
    }
  }

  return current_state_3d_cov;
}

// Sets the time stamp of current measurement and computes the time passed since last measurement
void CRadarTrackerMatlabWrapper::setTimeStampAndComputeDt(rclcpp::Time t_measurement/*, rclcpp::Clock& t_last_measurement*/)
{
  //! Berechne dt
  //! ros::Duration dt;
  // rclcpp::Duration dt{(rcl_duration_value_t)1.0};
  rclcpp::Duration dt{rclcpp::Duration(duration_1sec)};

  // m_t_last_measurement = t_last_measurement.now();

  // dt = rclcpp::Duration(0.070);

  // if(!m_t_last_measurement.isValid())
  // {
  //   dt = ros::Duration(0.070);
  // }
  // else
  // {
  //   dt = t_measurement - m_t_last_measurement;
  // }

  // // error: old timestamp invalid
  // if (dt > ros::Duration(0.5))
  // {
  //   // Clear Objektlist and Grid
  //   std::cout << boost::format("[CRadarTrackerMatlabWrapper::setTimeStampAndComputeDt] invalid timestamp: dt = %1.3f, using 0.07s instead!") % dt;

  //   dt = ros::Duration(0.070);
  // }

  // m_t_last_measurement = t_measurement;
  // m_current_dt = dt;
  // m_meas_list_st.t = t_measurement.toSec();


  //! if(!m_t_last_measurement.isValid())
  // if(!m_t_last_measurement.seconds() == 0) //TODO
  if(m_t_last_measurement.seconds() == 0) 
  {
    // dt = rclcpp::Duration(0.070);
    dt = rclcpp::Duration(duration_70ms);
  }
  else
  {
    dt = t_measurement - m_t_last_measurement;
  }

  // error: old timestamp invalid
  // if (dt > rclcpp::Duration(0.5))
  if (dt > rclcpp::Duration(duration_500ms))
  {
    // Clear Objektlist and Grid
    std::cout << boost::format("[CRadarTrackerMatlabWrapper::setTimeStampAndComputeDt] invalid timestamp: dt = %1.3f, using 0.07s instead!") % dt.seconds();

    // dt = rclcpp::Duration(0.070);
    dt = rclcpp::Duration(duration_70ms);
  }

  m_t_last_measurement = t_measurement;
  m_current_dt = dt;
  //! m_meas_list_st.t = t_measurement.toSec();
  m_meas_list_st.t = t_measurement.seconds();
}

void CRadarTrackerMatlabWrapper::clearMeasurementData()
{
  // m_meas_list_st = {0}; warning: extended initializer lists only available with -std=c++11 or -std=gnu++11
  memset(&m_meas_list_st, 0, sizeof(m_meas_list_st));
}

#if TF_SWITCH_ON
void CRadarTrackerMatlabWrapper::setSensorTransform(tf2::Transform transformSensor)
{
  m_transformSensor = transformSensor;

  tf2::Vector3 vTranslation = m_transformSensor.getOrigin();
  tf2::Quaternion qRotation = m_transformSensor.getRotation();

  tf2::Matrix3x3 mRotation(qRotation);
  double roll, pitch, yaw;
  mRotation.getRPY(roll, pitch, yaw);

  m_meas_list_st.dx_sens_offset = vTranslation[0];
  m_meas_list_st.dy_sens_offset = vTranslation[1];
  m_meas_list_st.ang_sens_offset = yaw;
}
#endif

void CRadarTrackerMatlabWrapper::setLocationData(
  std::size_t locIdx,
  bool Measured,
  double dMeas,
  double dVarMeas,
  double vMeas,
  double vVarMeas,
  double dbRCS,
  double alpAng,
  double alpVarAng)
{
  m_meas_list_st.meas[locIdx] = Measured;
  m_meas_list_st.dr[locIdx] = dMeas;
  m_meas_list_st.drVar[locIdx] = dVarMeas;
  m_meas_list_st.vr[locIdx] = vMeas;
  m_meas_list_st.vrVar[locIdx] = vVarMeas;
  m_meas_list_st.alpVar[locIdx] = alpVarAng;
  m_meas_list_st.dBRcs[locIdx] = dbRCS;

  // Defaults for Gen5
  m_meas_list_st.PDH1[locIdx] = 0.95;
  m_meas_list_st.PDH0[locIdx] = 0.04;



  #if TF_SWITCH_ON
  // Transform alpha with sensor orientation
  tf2::Quaternion qLocationSensorCoordinates;
  qLocationSensorCoordinates.setRPY(0, 0, alpAng);

  // First apply the location angle, then transform this one into vehicle coordinates.
  // The order of application is right-to-left, see http://wiki.ros.org/tf2/Tutorials/Quaternions#Applying_a_quaternion_rotation
  tf2::Quaternion qLocationVehicleCoordinates = m_transformSensor.getRotation() * qLocationSensorCoordinates;

  tf2::Matrix3x3 mRotation(qLocationVehicleCoordinates);
  double roll, pitch, yaw;
  mRotation.getRPY(roll, pitch, yaw);

  m_meas_list_st.alpha[locIdx] = yaw;
  #endif

  #if !TF_SWITCH_ON
  m_meas_list_st.alpha[locIdx] = alpAng;
  #endif

}

void CRadarTrackerMatlabWrapper::setEgoData(
    double vx,
    double ax,
    double psiDt,
    double kapCurvTraj,
    double alpSideSlipAngle)
{
  m_meas_list_st.vx_ego = vx;
  m_meas_list_st.ax_ego = ax;
  m_meas_list_st.psiDt_ego = psiDt;
  m_meas_list_st.kapCurvTraj = kapCurvTraj;
  m_meas_list_st.beta_ego = alpSideSlipAngle;
}

void CRadarTrackerMatlabWrapper::setFovData(
    double fov_angle_elem,
    double fov_range_elem,
    std::size_t elems_ind)
{
  m_meas_list_st.fov_angle[elems_ind] = fov_angle_elem;
  m_meas_list_st.fov_range[elems_ind] = fov_range_elem;
}
