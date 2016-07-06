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

#define MAX_ACM_DEVS 20

#include <bot_core/bot_core.h>

#define TO_DEGREES(rad) ((rad)*180/M_PI)

#include "liburg/include/urg_utils.h"
#include "liburg/include/urg_sensor.h"
#include "liburg/samples/open_urg_sensor.h"

// Useful for printing the scan data.
static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
#if 1
    int front_index;

    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂��\��
    front_index = urg_step2index(urg, 0);
    printf("Distance from front of LIDAR is %ld [mm] @ timestamp %ld [msec].\n", data[front_index], time_stamp);

#else
    (void)time_stamp;

    int i;
    long min_distance;
    long max_distance;

    // \~japanese �S�Ẵf�[�^�� X-Y �̈ʒu���\��
    urg_distance_min_max(urg, &min_distance, &max_distance);
    for (i = 0; i < data_n; ++i) {
        long l = data[i];
        double radian;
        long x;
        long y;

        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }
        radian = urg_index2rad(urg, i);
        x = (long)(l * cos(radian));
        y = (long)(l * sin(radian));
        printf("(%ld, %ld), ", x, y);
    }
    printf("\n");
#endif
}

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
    int skip_scan = 9; // This is the max that it can be.

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

    // Begin polling the laserscanner for data.
    bool catastrophicError = !!urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, skip_scan);
    long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    long timestamp;
    // Retrieve Distance data for all eternity

    while (!catastrophicError){
      int n = urg_get_distance(&urg, data, &timestamp);
      catastrophicError = (n<0);
      // Debug
      print_data(&urg, data, n, timestamp);
    }


    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;


  // When killed:
    lcm_destroy(lcm);
    //
    // urg_laserOff(&urg);
    // urg_disconnect(&urg);


    // if(sync)
    //     bot_timestamp_sync_free(sync);
    // free(data);
    // free(intensity);
    // free(lcm_url);
    // free(channel);
    // free(device);

    // return exit_code;
}
