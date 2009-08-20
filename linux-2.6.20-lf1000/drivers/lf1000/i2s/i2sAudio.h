#ifndef I2SAUDIO_H_
#define I2SAUDIO_H_

#include <linux/platform_device.h>
#include <asm/arch/common.h>

int __init initAudio(struct resource *);
AUDIO_REGS *getAudioRegs(void);
void setAudioPcmOutBuffer(u8 enableFlag);
void deinitAudio(struct resource *res);

#endif /*I2SAUDIO_H_*/
