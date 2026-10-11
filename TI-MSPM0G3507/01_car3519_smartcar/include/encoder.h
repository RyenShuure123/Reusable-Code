#ifndef CAR3519NEW_ENCODER_H
#define CAR3519NEW_ENCODER_H
#include <stdint.h>

void Encoder_Init(void);
void Encoder_Reset(void);
int16_t Read_Encoder_Left(void);
int16_t Read_Encoder_Right(void);

#endif //CAR3519NEW_ENCODER_H
