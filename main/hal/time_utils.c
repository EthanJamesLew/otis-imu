#include "time_utils.h"

uint32_t get_time_millis () {
    return (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
}

uint32_t get_time_micros () {
    return (uint32_t) (clock() * 1000000 / CLOCKS_PER_SEC);
}

void start_hal_timer(timer_hal_t* timer){
    timer->curr = get_time_micros();
    timer->prev = 0;
    timer->diff = timer->curr - timer->prev;
}
 

void update_hal_timer(timer_hal_t* timer){
    timer->curr = get_time_micros();
    timer->diff = timer->curr - timer->prev;
}

void reset_hal_timer(timer_hal_t* timer){
    timer->prev = timer->curr;
    timer->curr = get_time_micros();
    timer->diff = timer->curr - timer->prev;
}