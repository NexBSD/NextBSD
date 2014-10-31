
#ifndef	_UKERN_DEVICE_H_
#define	_UKERN_DEVICE_H_


int device_get_fd(device_t);

int device_get_irq(device_t dev, int vector);

#endif
