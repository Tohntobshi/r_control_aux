#include "utils.h"
#include "lwip/def.h"

float get_float_from_net(uint8_t * data) {
    uint32_t tmp = ntohl(*(uint32_t *)(data));
    return *(float *)(&tmp);
}

uint16_t get_short_from_net(uint8_t * data) {
    return ntohs(*(uint16_t*)(data));
}

int get_int_from_net(uint8_t * data) {
    return ntohl(*(uint32_t*)(data));
}

void set_float_to_net(float val, uint8_t * dest) {
    *(uint32_t *)dest = htonl(*(uint32_t *)(&val));
}

void set_short_to_net(uint16_t val, uint8_t * dest) {
    *(uint16_t *)dest = htons(val);
}

void set_int_to_net(uint32_t val, uint8_t * dest) {
    *(uint32_t *)dest = htonl(val);
}