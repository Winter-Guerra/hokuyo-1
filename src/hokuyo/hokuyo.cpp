/**
 * LCM module for connecting to a Hokuyo laser range finder (URG and UTM are
 * supported) and transmitting range data via LCM.
 */

#include <sys/time.h>
#include <time.h>

#include <glib.h>
#include <lcm/lcm.h>
#include <lcmtypes/lcmtypes.h>

#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>

using namespace qrk;
using namespace std;


#define SKIP_SCANS 0 // Number of scans to skip [0,9]
#define PUBLISHING_CHANNEL "scan"
#define TIMEOUT 1000 // in ms

//#include <bot_core/bot_core.h>

#define TO_DEGREES(rad) ((rad)*180/M_PI)


long int getPCms(void){
        struct timeval tp;
        gettimeofday(&tp, NULL);
        long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
}

// Where the magic happens
int main(int argc, char *argv[])
{
        setlinebuf(stdout);

        // setup LCM
        lcm_t *lcm = lcm_create(NULL);
        if (!lcm) {
                fprintf(stderr, "Couldn't setup LCM\n");
                return 1;
        }


        // Run the driver FOREVER
        while (TRUE) {
                printf("##############################\n");
                printf("Opening connection to Hokuyo\n");

                // Params from the laser scanner
                Urg_driver urg;
                int min_step;
                int max_step;
                long min_distance;
                long max_distance;
                long time_stamp_offset;

                // open the connection. Run with flags -e for ethernet conn.
                Connection_information information(argc, argv);

                if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type())) {
                        printf("urg.driver::open(): %s : %s", information.device_or_ip_name(), urg.what());
                }

                urg.set_timeout_msec(TIMEOUT);

                // sync the clocks
                urg.set_sensor_time_stamp(getPCms());

                // Print some information about the laser scanner
                printf("Sensor product type: %s\n", urg.product_type());
                printf("Sensor firmware version: %s\n", urg.firmware_version());
                printf("Sensor serial ID: %s\n", urg.serial_id());
                printf("Sensor status: %s\n", urg.status());
                printf("Sensor state: %s\n", urg.state());
                min_step = urg.min_step();
                max_step = urg.max_step();
                printf("step: [%d, %d]\n", min_step, max_step);
                // urg.distance_min_max(&urg, &min_distance, &max_distance);
                // printf("distance: [%ld, %ld)\n", min_distance, max_distance);
                printf("scan interval: %ld [usec]\n", urg.scan_usec());
                printf("sensor data size: %d\n", urg.max_data_size());
                // printf("timestamp offset: %ld\n", time_stamp_offset);


                // @TODO: set timestamp of lidar to that of computer epoch.

                // Begin polling the laserscanner for intensity data.
                bool catastrophicError = !urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, SKIP_SCANS);


                // error handling
                if (catastrophicError) {
                        printf("Catastrophic error from URG %s\n", urg.what());
                        // printf("Error #%d\n",urg.last_errno);
                        continue;
                };

                vector<long> data;
                vector<short unsigned int> intensities;
                long timestamp;

                // Precreate the message structure
                bot_core_planar_lidar_t msg;
                // the angle (in radians) to the first point in nranges,
                // relative to the laser scanner's own coordinate frame.
                msg.rad0 = urg.step2rad(min_step);
                printf("Starting radian: %f\n", msg.rad0);
                // the number of radians between each successive sample
                msg.radstep = (urg.step2rad(1) - urg.step2rad(0));
                printf("radstep: %f\n", msg.radstep);
                // range data (meters)
                msg.nranges = urg.max_data_size();
                msg.ranges = (float*) malloc(sizeof(data[0]) * msg.nranges);
                // Intensity data
                msg.nintensities = urg.max_data_size();
                msg.intensities = (float*) malloc(sizeof(msg.intensities[0]) * msg.nintensities);

                // Retrieve Distance data for all eternity
                while (!catastrophicError) {
                        // Get the sensor reading
                        catastrophicError = !urg.get_distance_intensity(data, intensities, &timestamp);

                        // error printing
                        if (catastrophicError) {
                              printf("Error from URG %s\n", urg.what());
                                // printf("Error #%d\n",urg.last_errno);
                                // Restart the measurement process.
                              //   if (urg.errorno() == -8){
                              //   catastrophicError = !urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, SKIP_SCANS);
                              //
                              // }
                              // continue;
                              break;
                        }


                        // Output to LCM.

                        //  int64_t  utime;
                        msg.utime = timestamp;

                        // Copy and convert scan data from mm to m
                        int i;
                        for (i=0; i<data.size(); i+=1) {
                                // convert cm into meters
                                msg.ranges[i] = (data[i] * 10e-4);
                                // Convert intensity data to more or less match that of a SICK lidar
                                msg.intensities[i] = (intensities[i] *= 15.0);
                        }


                        // Publish the message
                        bot_core_planar_lidar_t_publish(lcm, PUBLISHING_CHANNEL, &msg);
                        // printf("published a message!\n");

                }

                // When killed:
                urg.close();
                #if defined(URG_MSC)
                  getchar();
                #endif
                // Delay for 25ms
                struct timespec timeOut,remains;
                timeOut.tv_sec = 0;
                timeOut.tv_nsec = 500*10+6;
                nanosleep(&timeOut, &remains);//100ms
        }

}
