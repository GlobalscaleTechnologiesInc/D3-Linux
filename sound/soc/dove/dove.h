/*
 * kirkwood.h
 *
 * (c) 2010 Arnaud Patard <apatard@mandriva.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _DOVE_AUDIO_H
#define _DOVE_AUDIO_H

#define DOVE_RECORD_WIN			0
#define DOVE_PLAYBACK_WIN			1
#define DOVE_MAX_AUDIO_WIN			2

#define DOVE_AUDIO_WIN_BASE_REG(win)	(0xA00 + ((win)<<3))
#define DOVE_AUDIO_WIN_CTRL_REG(win)	(0xA04 + ((win)<<3))


#define DOVE_RECCTL			0x1000
#define DOVE_RECCTL_SPDIF_EN		(1<<11)
#define DOVE_RECCTL_I2S_EN			(1<<10)
#define DOVE_RECCTL_PAUSE			(1<<9)
#define DOVE_RECCTL_MUTE			(1<<8)
#define DOVE_RECCTL_BURST_MASK		(3<<5)
#define DOVE_RECCTL_BURST_128		(2<<5)
#define DOVE_RECCTL_BURST_32		(1<<5)
#define DOVE_RECCTL_MONO			(1<<4)
#define DOVE_RECCTL_MONO_CHAN_RIGHT	(1<<3)
#define DOVE_RECCTL_MONO_CHAN_LEFT		(0<<3)
#define DOVE_RECCTL_SIZE_MASK		(7<<0)
#define DOVE_RECCTL_SIZE_16		(7<<0)
#define DOVE_RECCTL_SIZE_16_C		(3<<0)
#define DOVE_RECCTL_SIZE_20		(2<<0)
#define DOVE_RECCTL_SIZE_24		(1<<0)
#define DOVE_RECCTL_SIZE_32		(0<<0)

#define DOVE_REC_BUF_ADDR			0x1004
#define DOVE_REC_BUF_SIZE			0x1008
#define DOVE_REC_BYTE_COUNT			0x100C

#define DOVE_PLAYCTL			0x1100
#define DOVE_PLAYCTL_PLAY_BUSY		(1<<16)
#define DOVE_PLAYCTL_BURST_MASK		(3<<11)
#define DOVE_PLAYCTL_BURST_128		(2<<11)
#define DOVE_PLAYCTL_BURST_32		(1<<11)
#define DOVE_PLAYCTL_PAUSE			(1<<9)
#define DOVE_PLAYCTL_SPDIF_MUTE		(1<<8)
#define DOVE_PLAYCTL_MONO_MASK		(3<<5)
#define DOVE_PLAYCTL_MONO_BOTH		(3<<5)
#define DOVE_PLAYCTL_MONO_OFF		(0<<5)
#define DOVE_PLAYCTL_I2S_MUTE		(1<<7)
#define DOVE_PLAYCTL_SPDIF_EN		(1<<4)
#define DOVE_PLAYCTL_I2S_EN		(1<<3)
#define DOVE_PLAYCTL_SIZE_MASK		(7<<0)
#define DOVE_PLAYCTL_SIZE_16		(7<<0)
#define DOVE_PLAYCTL_SIZE_16_C		(3<<0)
#define DOVE_PLAYCTL_SIZE_20		(2<<0)
#define DOVE_PLAYCTL_SIZE_24		(1<<0)
#define DOVE_PLAYCTL_SIZE_32		(0<<0)

#define DOVE_PLAY_BUF_ADDR			0x1104
#define DOVE_PLAY_BUF_SIZE			0x1108
#define DOVE_PLAY_BYTE_COUNT		0x110C

#define DOVE_DCO_CTL			0x1204
#define DOVE_DCO_CTL_OFFSET_MASK		(0xFFF<<2)
#define DOVE_DCO_CTL_OFFSET_0		(0x800<<2)
#define DOVE_DCO_CTL_FREQ_MASK		(3<<0)
#define DOVE_DCO_CTL_FREQ_11		(0<<0)
#define DOVE_DCO_CTL_FREQ_12		(1<<0)
#define DOVE_DCO_CTL_FREQ_24		(2<<0)

#define DOVE_DCO_SPCR_STATUS		0x120c
#define DOVE_DCO_SPCR_STATUS_DCO_LOCK	(1<<16)

#define DOVE_ERR_CAUSE			0x1300
#define DOVE_ERR_MASK			0x1304

#define DOVE_INT_CAUSE			0x1308
#define DOVE_INT_MASK			0x130C
#define DOVE_INT_CAUSE_PLAY_BYTES		(1<<14)
#define DOVE_INT_CAUSE_REC_BYTES		(1<<13)
#define DOVE_INT_CAUSE_DMA_PLAY_END	(1<<7)
#define DOVE_INT_CAUSE_DMA_PLAY_3Q		(1<<6)
#define DOVE_INT_CAUSE_DMA_PLAY_HALF	(1<<5)
#define DOVE_INT_CAUSE_DMA_PLAY_1Q		(1<<4)
#define DOVE_INT_CAUSE_DMA_REC_END		(1<<3)
#define DOVE_INT_CAUSE_DMA_REC_3Q		(1<<2)
#define DOVE_INT_CAUSE_DMA_REC_HALF	(1<<1)
#define DOVE_INT_CAUSE_DMA_REC_1Q		(1<<0)

#define DOVE_REC_BYTE_INT_COUNT		0x1310
#define DOVE_PLAY_BYTE_INT_COUNT		0x1314
#define DOVE_BYTE_INT_COUNT_MASK		0xffffff

#define DOVE_I2S_PLAYCTL			0x2508
#define DOVE_I2S_RECCTL			0x2408
#define DOVE_I2S_CTL_JUST_MASK		(0xf<<26)
#define DOVE_I2S_CTL_LJ			(0<<26)
#define DOVE_I2S_CTL_I2S			(5<<26)
#define DOVE_I2S_CTL_RJ			(8<<26)
#define DOVE_I2S_CTL_SIZE_MASK		(3<<30)
#define DOVE_I2S_CTL_SIZE_16		(3<<30)
#define DOVE_I2S_CTL_SIZE_20		(2<<30)
#define DOVE_I2S_CTL_SIZE_24		(1<<30)
#define DOVE_I2S_CTL_SIZE_32		(0<<30)

#define DOVE_AUDIO_BUF_MAX			(16*1024*1024)

/* Theses values come from the marvell alsa driver */
/* need to find where they come from               */
#define DOVE_SND_MIN_PERIODS		8
#define DOVE_SND_MAX_PERIODS		16
#define DOVE_SND_MIN_PERIOD_BYTES		0x4000
#define DOVE_SND_MAX_PERIOD_BYTES		0x4000

struct kirkwood_dma_data {
	struct resource *mem;
	void __iomem *io;
	int irq;
	int burst;
};

#endif
