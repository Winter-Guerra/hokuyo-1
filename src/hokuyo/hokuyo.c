/**
 * LCM module for connecting to a Hokuyo laser range finder (URG and UTM are
 * supported) and transmitting range data via LCM.
 */

#include <stdio.h>
#include <stdlib.h>
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

int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    // setup LCM
    lcm_t *lcm = lcm_create(NULL);
    if(!lcm) {
        fprintf(stderr, "Couldn't setup LCM\n");
        return 1;
    }

    urg_t urg;
    int min_step;
    int max_step;
    long min_distance;
    long max_distance;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

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
