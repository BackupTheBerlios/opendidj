#ifndef ADC_PRIV_H
#define ADC_PRIV_H

struct adc_device {
	void *mem;
	int irq;
	spinlock_t lock;
	wait_queue_head_t wait;
	wait_queue_head_t meas_busy;
	int busy;
	int conversionFinished;
};

#endif
