#ifndef BATTERY_H
#define BATTERY_H

int battery_init(void);
int battery_measure_enable(bool enable);
int battery_sample(void);

#endif
