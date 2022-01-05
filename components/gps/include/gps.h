#include <stdint.h>

void gps_setup();
float get_latitude();
float get_longitude();
int get_num_satelites();
uint8_t get_lat_lon_actuality(); // should be called first