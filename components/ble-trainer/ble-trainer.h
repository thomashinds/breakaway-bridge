#ifndef BLE_TRAINER_H
#define BLE_TRAINER_H

void ble_init(void);


/* This function simulates power and notifies it to the client */
void
notify_power(int power_watts /* xTimerHandle ev */);

#endif