#ifndef LOOP_H
#define LOOP_H

#include <stdint.h>

#define LOOP_SENSOR_NUM 5
#define LOOP_LOST_LINE_TOLERANCE_CYCLES 3U

/* Most TCRT5000 boards output low when black is detected. */
#define LOOP_SENSOR_ACTIVE_LEVEL GPIO_PIN_RESET

typedef struct
{
    uint8_t value[LOOP_SENSOR_NUM];
    uint8_t active_count;
    uint8_t lost_line_cycles;
    int last_error;
} LoopSensor_t;

void Read_XJSensor(void);
int Get_Track_Error(void);
void Loop_VOFA_Debug_Send(void);

#endif
