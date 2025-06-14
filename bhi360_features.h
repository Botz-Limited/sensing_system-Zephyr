#ifndef BHI360_FEATURES_H
#define BHI360_FEATURES_H

// BHI360 Virtual Sensor Feature IDs
// Generated for easy access in your application

#define BHY2_SENSOR_ID_ACC_PASS                   1   // Accelerometer passthrough
#define BHY2_SENSOR_ID_ACC_RAW                    3   // Accelerometer uncalibrated
#define BHY2_SENSOR_ID_ACC                        4   // Accelerometer corrected
#define BHY2_SENSOR_ID_ACC_BIAS                   5   // Accelerometer offset
#define BHY2_SENSOR_ID_ACC_WU                     6   // Accelerometer corrected wake up
#define BHY2_SENSOR_ID_ACC_RAW_WU                 7   // Accelerometer uncalibrated wake up
#define BHY2_SENSOR_ID_SI_ACCEL                   8   // Virtual Sensor ID for Accelerometer
#define BHY2_SENSOR_ID_GYRO_PASS                  10  // Gyroscope passthrough
#define BHY2_SENSOR_ID_GYRO_RAW                   12  // Gyroscope uncalibrated
#define BHY2_SENSOR_ID_GYRO                       13  // Gyroscope corrected
#define BHY2_SENSOR_ID_GYRO_BIAS                  14  // Gyroscope offset
#define BHY2_SENSOR_ID_GYRO_WU                    15  // Gyroscope wake up
#define BHY2_SENSOR_ID_GYRO_RAW_WU                16  // Gyroscope uncalibrated wake up
#define BHY2_SENSOR_ID_SI_GYROS                   17  // Virtual Sensor ID for Gyroscope
#define BHY2_SENSOR_ID_MAG_PASS                   19  // Magnetometer passthrough
#define BHY2_SENSOR_ID_MAG_RAW                    21  // Magnetometer uncalibrated
#define BHY2_SENSOR_ID_MAG                        22  // Magnetometer corrected
#define BHY2_SENSOR_ID_MAG_BIAS                   23  // Magnetometer offset
#define BHY2_SENSOR_ID_MAG_WU                     24  // Magnetometer wake up
#define BHY2_SENSOR_ID_MAG_RAW_WU                 25  // Magnetometer uncalibrated wake up
#define BHY2_SENSOR_ID_GRA                        28  // Gravity vector
#define BHY2_SENSOR_ID_GRA_WU                     29  // Gravity vector wake up
#define BHY2_SENSOR_ID_LACC                       31  // Linear acceleration
#define BHY2_SENSOR_ID_LACC_WU                    32  // Linear acceleration wake up
#define BHY2_SENSOR_ID_RV                         34  // Rotation vector
#define BHY2_SENSOR_ID_RV_WU                      35  // Rotation vector wake up
#define BHY2_SENSOR_ID_GAMERV                     37  // Game rotation vector
#define BHY2_SENSOR_ID_GAMERV_WU                  38  // Game rotation vector wake up
#define BHY2_SENSOR_ID_GEORV                      40  // Geo-magnetic rotation vector
#define BHY2_SENSOR_ID_GEORV_WU                   41  // Geo-magnetic rotation vector wake up
#define BHY2_SENSOR_ID_ORI                        43  // Orientation
#define BHY2_SENSOR_ID_ORI_WU                     44  // Orientation wake up
#define BHY2_SENSOR_ID_TILT_DETECTOR              48  // Tilt detector
#define BHY2_SENSOR_ID_STD                        50  // Step detector
#define BHY2_SENSOR_ID_STC                        52  // Step counter
#define BHY2_SENSOR_ID_STC_WU                     53  // Step counter wake up
#define BHY2_SENSOR_ID_SIG                        55  // Significant motion
#define BHY2_SENSOR_ID_WAKE_GESTURE               57  // Wake gesture
#define BHY2_SENSOR_ID_GLANCE_GESTURE             59  // Glance gesture
#define BHY2_SENSOR_ID_PICKUP_GESTURE             61  // Pickup gesture
#define BHY2_SENSOR_ID_AR                         63  // Activity recognition
#define BHY2_SENSOR_ID_WRIST_TILT_GESTURE         67  // Wrist tilt gesture
#define BHY2_SENSOR_ID_DEVICE_ORI                 69  // Device orientation
#define BHY2_SENSOR_ID_DEVICE_ORI_WU              70  // Device orientation wake up
#define BHY2_SENSOR_ID_STATIONARY_DET             75  // Stationary detect
#define BHY2_SENSOR_ID_MOTION_DET                 77  // Motion detect
#define BHY2_SENSOR_ID_ACC_BIAS_WU                91  // Accelerometer offset wake up
#define BHY2_SENSOR_ID_GYRO_BIAS_WU               92  // Gyroscope offset wake up
#define BHY2_SENSOR_ID_MAG_BIAS_WU                93  // Magnetometer offset wake up
#define BHY2_SENSOR_ID_STD_WU                     94  // Step detector wake up
#define BHY2_SENSOR_ID_TEMP                       128 // Temperature
#define BHY2_SENSOR_ID_BARO                       129 // Barometer
#define BHY2_SENSOR_ID_HUM                        130 // Humidity
#define BHY2_SENSOR_ID_GAS                        131 // Gas
#define BHY2_SENSOR_ID_TEMP_WU                    132 // Temperature wake up
#define BHY2_SENSOR_ID_BARO_WU                    133 // Barometer wake up
#define BHY2_SENSOR_ID_HUM_WU                     134 // Humidity wake up
#define BHY2_SENSOR_ID_GAS_WU                     135 // Gas wake up
#define BHY2_SENSOR_ID_STC_LP                     136 // Step counter Low Power
#define BHY2_SENSOR_ID_STD_LP                     137 // Step detector Low Power
#define BHY2_SENSOR_ID_SIG_LP                     138 // Significant motion Low Power
#define BHY2_SENSOR_ID_STC_LP_WU                  139 // Step counter Low Power wake up
#define BHY2_SENSOR_ID_STD_LP_WU                  140 // Step detector Low Power wake up
#define BHY2_SENSOR_ID_SIG_LP_WU                  141 // Significant motion Low Power wake up
#define BHY2_SENSOR_ID_ANY_MOTION_LP              142 // Any motion Low Power
#define BHY2_SENSOR_ID_ANY_MOTION_LP_WU           143 // Any motion Low Power wake up
#define BHY2_SENSOR_ID_EXCAMERA                   144 // External camera trigger
#define BHY2_SENSOR_ID_GPS                        145 // GPS
#define BHY2_SENSOR_ID_LIGHT                      146 // Light
#define BHY2_SENSOR_ID_PROX                       147 // Proximity
#define BHY2_SENSOR_ID_LIGHT_WU                   148 // Light wake up
#define BHY2_SENSOR_ID_PROX_WU                    149 // Proximity wake up
#define BHY2_SENSOR_ID_KLIO                       112 // Klio
#define BHY2_SENSOR_ID_KLIO_LOG                   127 // Klio log
#define BHY2_SENSOR_ID_SWIM                       114 // Swim recognition
#define BHY2_SENSOR_ID_AIR_QUALITY                115 // Air Quality
#define BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG           120 // Head Orientation Misalignment
#define BHY2_SENSOR_ID_IMU_HEAD_ORI_Q             121 // IMU Head Orientation Quaternion
#define BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q            122 // NDOF Head Orientation Quaternion
#define BHY2_SENSOR_ID_IMU_HEAD_ORI_E             123 // IMU Head Orientation Euler
#define BHY2_SENSOR_ID_NDOF_HEAD_ORI_E            124 // NDOF Head Orientation Euler

#endif // BHI360_FEATURES_H
