/*! time.h
*  @author Ethan Lew
*
* Provide a very simple abstraction of timing functions.
*/

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <stdint.h>
#include <time.h>

typedef struct timer_hal_s{
    uint32_t prev;
    uint32_t curr;
    uint32_t diff; 
} timer_hal_t;

/*
* @brief get the time in milliseconds
*/
uint32_t get_time_millis();

/*
* @brief get the time in microseconds
*/
uint32_t get_time_micros ();

/*!
* @brief create a hal timer
*/
void start_hal_timer(timer_hal_t* timer);

/*!
* @brief update a hal timer
*/
void update_hal_timer(timer_hal_t* timer);

/*!
* @brief reset a hal timer
*/
void reset_hal_timer(timer_hal_t* timer);

#endif