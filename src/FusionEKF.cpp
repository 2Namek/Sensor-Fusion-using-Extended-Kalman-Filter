#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include<cmath>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  //cout<< "Initialization0 started"<<endl;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.z_pred_ = VectorXd(3);
  //measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

  //measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;
	

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //H of the laser which is only removing the other non measureable components
  //cout<< "Initialization started"<<endl;
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;
	
	Hj_ << 	1, 1, 0, 0,
			1, 1, 0, 0,
			1, 1, 1, 1;
	
		
	ekf_.F_ <<	1, 0, 1, 0,
				0, 1, 0, 1,
				0, 0, 1, 1,
				0, 0, 0, 1;
				
	ekf_.P_ <<	1, 0, 0, 	0,
				0, 1, 0, 	0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;		
	//ekf_.x_ << 	1,1,1,1;
	
	ekf_.Q_ <<	1, 0, 1, 0,
				0, 1, 0, 1,
				1, 0, 1, 0,
				0, 1, 0, 1;			

	
	noise_ax = 9;
	noise_ay = 9;

	  //cout<< "Initialization ended"<<endl;
	count =0;
   
	
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */	
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
	ekf_.x_ << 1, 1, 1, 1;
	//ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
	cout<< "2nd step not initialized"<<endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
	  //cout<<ekf_.x_<<endl;
    }
	
    // done initializing, no need to predict or update
	
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
	cout<<is_initialized_<<endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   count++;
   cout<<count<<endl;
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	ekf_.F_ << 1, 0, dt, 0,
			   0, 1, 0, dt,
			   0, 0, 1,  0,
			   0, 0, 0,  1;

	
	ekf_.Q_ << (pow(noise_ax,2)*pow(dt,4))/4, 0, (pow(noise_ax,2)*pow(dt,3))/2, 0,
			  0,(pow(noise_ay,2)*pow(dt,4))/4, 0, (pow(noise_ay,2)*pow(dt,3))/2,
			  (pow(noise_ax,2)*pow(dt,3))/2, 0, pow(noise_ax,2)*pow(dt,2), 0,
			  0, (pow(noise_ay,2)*pow(dt,3))/2, 0, pow(noise_ay,2)*pow(dt,2);		  
			  
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
   
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	//cout<<"RADAR Reading"<<endl;
	
	cout<<ekf_.x_<<endl;
	pxx = ekf_.x_(0);
   pyy = ekf_.x_(1);
   vxx = ekf_.x_(2);
   vyy = ekf_.x_(3);
  // if(pxx)
	   
   Hj_ <<   (pxx/(sqrt(pow(pxx,2)+pow(pyy,2)))) , (pyy/(sqrt(pow(pxx,2)+pow(pyy,2)))) , 0 , 0 ,
            (-pyy/(pow(pxx,2)+pow(pyy,2))) , (pxx/(pow(pxx,2)+pow(pyy,2))) , 0 , 0 ,
            ((pyy*(vxx*pyy-vyy*pxx))/(pow((pow(pxx,2)+pow(pyy,2)),1.5))) , ((pxx*(vyy*pxx-vxx*pyy))/(pow((pow(pxx,2)+pow(pyy,2)),1.5))) , (pxx/(sqrt(pow(pxx,2)+pow(pyy,2)))) , (pyy/(sqrt(pow(pxx,2)+pow(pyy,2))));
	ekf_.H_ = 	Hj_;
	ekf_.R_ = R_radar_;
	// cout<<"logpxx" <<pxx << " " << pyy << " " << vxx << " " << vyy << endl;
	 double phi = (sqrt(pow(pxx,2)+pow(pyy,2)));
	 double theta = atan2(pyy,pxx);
	// cout<<"theta="<<theta*180/3.14159265<<endl;
	 double phi_dot = (((pxx*vxx)+(pyy*vyy))/(sqrt(pow(pxx,2)+pow(pyy,2))));
	ekf_.z_pred_ <<  phi , theta, phi_dot;
	//cout<<"logh(x')" <<ekf_.z_pred_<<endl;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }
  else {
    // Laser updates
	cout<<"Laser Reading"<<endl;
	ekf_.H_ = 	H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout<<"------------------------------------"<<endl;
}
