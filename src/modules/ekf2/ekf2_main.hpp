/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* Header file of ekf2_main.cpp
* @author Daniel Jáuregui
*/

#pragma once

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include <ecl/EKF/ekf.h>


class Ekf2 : public control::SuperBlock
{
public:
        /** Constructor */
        Ekf2();

        /** Destructor, also kills the task */
        ~Ekf2();

        /** Start task.
         * @return OK on success. */
        int start();

        void set_replay_mode(bool replay);

        static void task_main_trampoline(int argc, char *argv[]);

        void task_main();

        void print_status();

        void exit();

private:
        static constexpr float _dt_max = 0.02;
        bool	_task_should_exit = false;
        int	_control_task = -1;		// task handle for task
        bool 	_replay_mode;			// should we use replay data from a log
        int 	_publish_replay_mode;		// defines if we should publish replay messages
        float	_default_ev_pos_noise = 0.05f;	// external vision position noise used when an invalid value is supplied
        float	_default_ev_ang_noise = 0.05f;	// external vision angle noise used when an invalid value is supplied

        // Initialise time stamps used to send sensor data to the EKF and for logging
        uint64_t _timestamp_mag_us = 0;
        uint64_t _timestamp_balt_us = 0;

        // Used to down sample magnetometer data
        float _mag_data_sum[3];			// summed magnetometer readings (Ga)
        uint64_t _mag_time_sum_ms;		// summed magnetoemter time stamps (msec)
        uint8_t _mag_sample_count = 0;		// number of magnetometer measurements summed
        uint32_t _mag_time_ms_last_used = 0;	// time stamp in msec of the last averaged magnetometer measurement used by the EKF

        // Used to down sample barometer data
        float _balt_data_sum;			// summed barometric altitude readings (m)
        uint64_t _balt_time_sum_ms;		// summed barometric altitude time stamps (msec)
        uint8_t _balt_sample_count = 0;		// number of barometric altitude measurements summed
        uint32_t _balt_time_ms_last_used =
                0;	// time stamp in msec of the last averaged barometric altitude measurement used by the EKF

        bool	_prev_landed = true;	// landed status from the previous frame

        float _acc_hor_filt = 0.0f; 	// low-pass filtered horizontal acceleration

        orb_advert_t _att_pub;
        orb_advert_t _lpos_pub;
        orb_advert_t _control_state_pub;
        orb_advert_t _vehicle_global_position_pub;
        orb_advert_t _wind_pub;
        orb_advert_t _estimator_status_pub;
        orb_advert_t _estimator_innovations_pub;
        orb_advert_t _replay_pub;

        /* Low pass filter for attitude rates */
        math::LowPassFilter2p _lp_roll_rate;
        math::LowPassFilter2p _lp_pitch_rate;
        math::LowPassFilter2p _lp_yaw_rate;

        Ekf _ekf;

        parameters *_params;	// pointer to ekf parameter struct (located in _ekf class instance)

        control::BlockParamExtInt _obs_dt_min_ms;
        control::BlockParamExtFloat _mag_delay_ms;
        control::BlockParamExtFloat _baro_delay_ms;
        control::BlockParamExtFloat _gps_delay_ms;
        control::BlockParamExtFloat _flow_delay_ms;
        control::BlockParamExtFloat _rng_delay_ms;
        control::BlockParamExtFloat _airspeed_delay_ms;
        control::BlockParamExtFloat _ev_delay_ms;

        control::BlockParamExtFloat _gyro_noise;
        control::BlockParamExtFloat _accel_noise;

        // process noise
        control::BlockParamExtFloat _gyro_bias_p_noise;
        control::BlockParamExtFloat _accel_bias_p_noise;
        control::BlockParamExtFloat _mage_p_noise;
        control::BlockParamExtFloat _magb_p_noise;
        control::BlockParamExtFloat _wind_vel_p_noise;
        control::BlockParamExtFloat _terrain_p_noise;	// terrain offset state random walk (m/s)
        control::BlockParamExtFloat _terrain_gradient;	// magnitude of terrain gradient (m/m)

        control::BlockParamExtFloat _gps_vel_noise;
        control::BlockParamExtFloat _gps_pos_noise;
        control::BlockParamExtFloat _pos_noaid_noise;
        control::BlockParamExtFloat _baro_noise;
        control::BlockParamExtFloat _baro_innov_gate;	// innovation gate for barometric height innovation test (std dev)
        control::BlockParamExtFloat
        _posNE_innov_gate;    // innovation gate for GPS horizontal position innovation test (std dev)
        control::BlockParamExtFloat _vel_innov_gate;	// innovation gate for GPS velocity innovation test (std dev)
        control::BlockParamExtFloat _tas_innov_gate;	// innovation gate for tas innovation test (std dev)

        control::BlockParamExtFloat _mag_heading_noise;	// measurement noise used for simple heading fusion
        control::BlockParamExtFloat _mag_noise;		// measurement noise used for 3-axis magnetoemter fusion (Gauss)
        control::BlockParamExtFloat _eas_noise;		// measurement noise used for airspeed fusion (std m/s)
        control::BlockParamExtFloat _beta_noise;	// synthetic sideslip noise (m/s)
        control::BlockParamExtFloat _mag_declination_deg;// magnetic declination in degrees
        control::BlockParamExtFloat _heading_innov_gate;// innovation gate for heading innovation test
        control::BlockParamExtFloat _mag_innov_gate;	// innovation gate for magnetometer innovation test
        control::BlockParamExtInt
        _mag_decl_source;       // bitmasked integer used to control the handling of magnetic declination
        control::BlockParamExtInt _mag_fuse_type;         // integer ued to control the type of magnetometer fusion used

        control::BlockParamExtInt _gps_check_mask;	// bitmasked integer used to activate the different GPS quality checks
        control::BlockParamExtFloat _requiredEph;	// maximum acceptable horiz position error (m)
        control::BlockParamExtFloat _requiredEpv;	// maximum acceptable vert position error (m)
        control::BlockParamExtFloat _requiredSacc;	// maximum acceptable speed error (m/s)
        control::BlockParamExtInt _requiredNsats;	// minimum acceptable satellite count
        control::BlockParamExtFloat _requiredGDoP;	// maximum acceptable geometric dilution of precision
        control::BlockParamExtFloat _requiredHdrift;	// maximum acceptable horizontal drift speed (m/s)
        control::BlockParamExtFloat _requiredVdrift;	// maximum acceptable vertical drift speed (m/s)
        control::BlockParamExtInt _param_record_replay_msg;// indicates if we want to record ekf2 replay messages

        // measurement source control
        control::BlockParamExtInt
        _fusion_mode;		// bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
        control::BlockParamExtInt _vdist_sensor_type;	// selects the primary source for height data

        // range finder fusion
        control::BlockParamExtFloat _range_noise;		// observation noise for range finder measurements (m)
        control::BlockParamExtFloat _range_innov_gate;	// range finder fusion innovation consistency gate size (STD)
        control::BlockParamExtFloat _rng_gnd_clearance;	// minimum valid value for range when on ground (m)

        // vision estimate fusion
        control::BlockParamExtFloat _ev_pos_noise;		// default position observation noise for exernal vision measurements (m)
        control::BlockParamExtFloat _ev_ang_noise;		// default angular observation noise for exernal vision measurements (rad)
        control::BlockParamExtFloat _ev_innov_gate;	// external vision position innovation consistency gate size (STD)

        // optical flow fusion
        control::BlockParamExtFloat
        _flow_noise;		// best quality observation noise for optical flow LOS rate measurements (rad/sec)
        control::BlockParamExtFloat
        _flow_noise_qual_min;	// worst quality observation noise for optical flow LOS rate measurements (rad/sec)
        control::BlockParamExtInt _flow_qual_min;		// minimum acceptable quality integer from  the flow sensor
        control::BlockParamExtFloat _flow_innov_gate;	// optical flow fusion innovation consistency gate size (STD)
        control::BlockParamExtFloat _flow_rate_max;	// maximum valid optical flow rate (rad/sec)

        // sensor positions in body frame
        control::BlockParamExtFloat _imu_pos_x;	// X position of IMU in body frame (m)
        control::BlockParamExtFloat _imu_pos_y;	// Y position of IMU in body frame (m)
        control::BlockParamExtFloat _imu_pos_z;	// Z position of IMU in body frame (m)
        control::BlockParamExtFloat _gps_pos_x;	// X position of GPS antenna in body frame (m)
        control::BlockParamExtFloat _gps_pos_y;	// Y position of GPS antenna in body frame (m)
        control::BlockParamExtFloat _gps_pos_z;	// Z position of GPS antenna in body frame (m)
        control::BlockParamExtFloat _rng_pos_x;	// X position of range finder in body frame (m)
        control::BlockParamExtFloat _rng_pos_y;	// Y position of range finder in body frame (m)
        control::BlockParamExtFloat _rng_pos_z;	// Z position of range finder in body frame (m)
        control::BlockParamExtFloat _flow_pos_x;	// X position of optical flow sensor focal point in body frame (m)
        control::BlockParamExtFloat _flow_pos_y;	// Y position of optical flow sensor focal point in body frame (m)
        control::BlockParamExtFloat _flow_pos_z;	// Z position of optical flow sensor focal point in body frame (m)
        control::BlockParamExtFloat _ev_pos_x;	// X position of VI sensor focal point in body frame (m)
        control::BlockParamExtFloat _ev_pos_y;	// Y position of VI sensor focal point in body frame (m)
        control::BlockParamExtFloat _ev_pos_z;	// Z position of VI sensor focal point in body frame (m)
        // control of airspeed and sideslip fusion
        control::BlockParamFloat
        _arspFusionThreshold; 	// a value of zero will disabled airspeed fusion. Any another positive value will determine
        // the minimum airspeed which will still be fused
        control::BlockParamInt _fuseBeta; // 0 disables synthetic sideslip fusion, 1 activates it

        // output predictor filter time constants
        control::BlockParamExtFloat _tau_vel;	// time constant used by the output velocity complementary filter (s)
        control::BlockParamExtFloat _tau_pos;	// time constant used by the output position complementary filter (s)

        // IMU switch on bias paameters
        control::BlockParamExtFloat _gyr_bias_init;	// 1-sigma gyro bias uncertainty at switch-on (rad/sec)
        control::BlockParamExtFloat _acc_bias_init;	// 1-sigma accelerometer bias uncertainty at switch-on (m/s**2)
        control::BlockParamExtFloat _ang_err_init;		// 1-sigma uncertainty in tilt angle after gravity vector alignment (rad)

        // airspeed mode parameter
        control::BlockParamInt _airspeed_mode;

        int update_subscriptions();

};
