#include <stdint.h>

float get_float_from_net(uint8_t * data);
uint16_t get_short_from_net(uint8_t * data);
int get_int_from_net(uint8_t * data);
void set_float_to_net(float val, uint8_t * dest);
void set_short_to_net(uint16_t val, uint8_t * dest);
void set_int_to_net(uint32_t val, uint8_t * dest);