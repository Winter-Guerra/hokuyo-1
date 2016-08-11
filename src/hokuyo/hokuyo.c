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

    // Open the laser scanner
    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

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

    // @TODO: set timestamp of lidar to that of computer epoch.

    // Begin polling the laserscanner for data.
    bool catastrophicError = !!urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, SKIP_SCANS);
    long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
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
    msg.ranges = (float*) malloc(sizeof(float) * msg.nranges);
    // Intensity data
    msg.nintensities = 0;
    msg.intensities = (float*) malloc(sizeof(float) * msg.nintensities);

    // Retrieve Distance data for all eternity
    while (!catastrophicError){
      // Get the sensor reading
      int n = urg_get_distance(&urg, data, &timestamp);
      catastrophicError = (n<0);

      // Debug
      // print_data(&urg, data, n, timestamp);

      // Output to LCM.

      // 	int64_t  utime;
      msg.utime = timestamp;

      // Copy and convert scan data from mm to m
      int i;
      for (i=0; i<n; i+=1) {
        msg.ranges[i] = (data[i] * 10e-3);
        // msg.ranges[i] = (data[i]);
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
    return 0;
    lcm_destroy(lcm);

}

// Useful for printing the scan data.
static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
    int front_index;

    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂��\��
    front_index = urg_step2index(urg, 0);
    printf("Distance from front of LIDAR is %ld [mm] @ timestamp %ld [msec].\n", data[front_index], time_stamp);
}
