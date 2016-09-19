#ifndef LQR_H_
#define LQR_H_

#include <stdbool.h>
#include <stdint.h>

#define mult_ut 1000
#define inv_dct 38.4615
#define inv_cq 34722
#define gain_wx 244949
#define gain_wy 244949
#define gain_wz 1
#define angle_gain_x 244949
#define angle_gain_y 244949
#define angle_gain_z 10
#define Max_Value 65535

void lqrInit(void);

bool lqrTest(void);

#endif/* LQR_H_ */