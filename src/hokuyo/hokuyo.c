/**
 * LCM module for connecting to a Hokuyo laser range finder (URG and UTM are
 * supported) and transmitting range data via LCM.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <sys/time.h>
#include <time.h>

#include <glib.h>
#include <lcm/lcm.h>


#define SKIP_SCANS 0 // Number of scans to skip [0,9]
#define PUBLISHING_CHANNEL "scan"

#include <bot_core/bot_core.h>

#define TO_DEGREES(rad) ((rad)*180/M_PI)

#include "liburg/include/urg_utils.h"
#include "liburg/include/urg_sensor.h"
#include "liburg/samples/open_urg_sensor.h"

long int getPCms(void){
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
}

// Useful for printing the scan data.
static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
    int front_index;

    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂��\��
    front_index = urg_step2index(urg, 0);
    printf("Distance from front of LIDAR is %ld [cm] @ timestamp %ld [msec].\n", data[front_index], time_stamp);
}

// Where the magic happens
int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    // setup LCM
    lcm_t *lcm = lcm_create(NULL);
    if(!lcm) {
        fprintf(stderr, "Couldn't setup LCM\n");
        return 1;
    }

    // Params from the laser scanner
    urg_t urg;
    int min_step;
    int max_step;
    long min_distance;
    long max_distance;
    long time_stamp_offset;

    // Open the laser scanner
    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    // sync the clocks
    urg_start_time_stamp_mode(&urg);
    time_stamp_offset = (getPCms() - urg_time_stamp(&urg));
    urg_stop_time_stamp_mode(&urg);

    // Print some information about the laser scanner
    printf("Sensor product type: %s\n", urg_sensor_product_type(&urg));
    printf("Sensor firmware version: %s\n", urg_sensor_firmware_version(&urg));
    printf("Sensor serial ID: %s\n", urg_sensor_serial_id(&urg));
    printf("Sensor status: %s\n", urg_sensor_status(&urg));
    printf("Sensor state: %s\n", urg_sensor_state(&urg));
    urg_step_min_max(&urg, &min_step, &max_step);
    printf("step: [%d, %d]\n", min_step, max_step);
    urg_distance_min_max(&urg, &min_distance, &max_distance);
    printf("distance: [%ld, %ld)\n", min_distance, max_distance);
    printf("scan interval: %ld [usec]\n", urg_scan_usec(&urg));
    printf("sensor data size: %d\n", urg_max_data_size(&urg));
    printf("timestamp offset: %ld\n", time_stamp_offset);


    // @TODO: set timestamp of lidar to that of computer epoch.

    // Begin polling the laserscanner for intensity data.
    bool catastrophicError = !!urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, URG_SCAN_INFINITY, SKIP_SCANS);

    // error handling
    if (catastrophicError){
      printf("Catastrophic error from URG %s\n", urg_error(&urg));
      printf("Error #%d\n",urg.last_errno);
      return urg.last_errno;
    }

    long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    unsigned short *intensities = malloc(urg_max_data_size(&urg) * sizeof(intensities[0]));
    long timestamp;

    // Precreate the message structure
    bot_core_planar_lidar_t msg;
    // the angle (in radians) to the first point in nranges,
    // relative to the laser scanner's own coordinate frame.
    msg.rad0 = urg_step2rad(&urg, min_step);
    printf("Starting radian: %f\n", msg.rad0);
    // the number of radians between each successive sample
    msg.radstep = (urg_step2rad(&urg, 1) - urg_step2rad(&urg, 0));
    printf("radstep: %f\n", msg.radstep);
    // range data (meters)
    msg.nranges = urg_max_data_size(&urg);
    msg.ranges = (float*) malloc(sizeof(data[0]) * msg.nranges);
    // Intensity data
    msg.nintensities = urg_max_data_size(&urg);
    msg.intensities = malloc(sizeof(msg.intensities[0]) * msg.nintensities);

    // Retrieve Distance data for all eternity
    while (!catastrophicError){
      // Get the sensor reading
      int n = urg_get_distance_intensity(&urg, data, intensities, &timestamp);

      // Check for errors in reading. Kill driver if something bad happens. Allow for procman to handle the restart.
      catastrophicError = (n<0);

      // error printing
      if (catastrophicError){
        printf("Error from URG %s\n", urg_error(&urg));
        printf("Error #%d\n",urg.last_errno);
        break;
      }


      // Output to LCM.

      // 	int64_t  utime;
      msg.timestampJetson = timestamp+time_stamp_offset;

      // Copy and convert scan data from mm to m
      int i;
      for (i=0; i<n; i+=1) {
        // convert cm into meters
        msg.ranges[i] = (data[i] * 10e-4);
        // Convert intensity data to more or less match that of a SICK lidar
        msg.intensities[i] = (intensities[i])*15.0;
      }

      // Publish the message
      bot_core_planar_lidar_t_publish(lcm, PUBLISHING_CHANNEL, &msg);
      // printf("published a message!\n");

    }



    // When killed:
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif

    lcm_destroy(lcm);
    return catastrophicError;

}
