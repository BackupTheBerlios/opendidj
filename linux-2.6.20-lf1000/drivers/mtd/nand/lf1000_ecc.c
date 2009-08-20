/*
 *  drivers/mtd/nand/lf1000_ecc.c
 *
 *  Andrey Yurovsky <andrey@cozybit.com>
 *  Robert Dowling <rdowling@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/nand.h>
#include <asm/io.h>
#include <asm/sizes.h>

/* control registers */
#define NAND_BASE	IO_ADDRESS(LF1000_MCU_S_BASE)

#if defined CPU_LF1000 && defined CONFIG_MTD_NAND_LF1000_HWECC

/*
 * The LF1000 hardware ECC is always enabled so no initialization is needed 
 * here.
 *
 * nand_read_page_syndrome() uses this function as follows:
 * - calls with NAND_ECC_READ, we need to grab the ECC and put it into ORGECC
 * - reads ecc.size bytes
 * - calls with NAND_ECC_READSYN, we need to wait for the decoder to finish
 *
 * nand_write_page_syndrome() uses this function as follows:
 * - calls with NAND_ECC_WRITE, we don't have to do anything
 * - writes ecc.size bytes
 * - (calls lf1000_calculate_ecc(), where we can wait for encoder to finish)
 */ 
void lf1000_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u32 x, s7, s5, s3, s1;

	switch(mode) {
	case NAND_ECC_READ:
		/* TODO: 'seek' to the right spot in the OOB, retrieve the ECC data
		 * 	 and place into the ORGECC registers and then reset the column
		 * 	 address for the normal write that will take place. */
		/* XXX: for now, we will just zero out the ORGECC */
		printk (KERN_DEBUG "!!! lf1000_enable_hwecc(NAND_ECC_READ)\n");

		/* Reset the HW ECC block (without changing the interrupts pending)
		 * Note: This was commented out of the VTK demo code, but it seems to help make writes consistent */
		x = readl(NAND_BASE+NFCONTROL);
		x &= ~((1UL << ECCRST) | (1UL << IRQPEND));
		x |= (1UL << ECCRST);
		writel (x, NAND_BASE+NFCONTROL);

		/* WORK IN PROGRESS... Fake out ECC for known patterns for debugging */
		/* Stuff fake ECC code for now */
		/* writel(0xd59e1ce9, NAND_BASE+NFORGECCL);	* for all ff data */
		/* writel(0x000b92b9, NAND_BASE+NFORGECCH);	* for all ff data */
		   writel(0xcdfb7ec1, NAND_BASE+NFORGECCL);	/* for 1234567890 then all ff data */
		   writel(0x000a0189, NAND_BASE+NFORGECCH);	/* for 1234567890 then all ff data */
		/* writel(0x2dd2bac0, NAND_BASE+NFORGECCL);	* for 1334567890 then all ff data */
		/* writel(0x000a1fd2, NAND_BASE+NFORGECCH);	* for 1334567890 then all ff data */
		break;

	case NAND_ECC_WRITE:
		printk (KERN_DEBUG "!!! lf1000_enable_hwecc(NAND_ECC_WRITE)\n");
		/* Reading NAND_BASE+NFCNT does not help...
		 *  x = readw(NAND_BASE+NFCNT); printk (KERN_DEBUG "    NFCNT = %08x\n", x); */
		/* Reset the HW ECC block (without changing the interrupts pending)
		 * Note: This was commented out of the VTK demo code, but it seems to help make writes consistent */
		x = readl(NAND_BASE+NFCONTROL);
		x &= ~((1UL << ECCRST) | (1UL << IRQPEND));
		x |= (1UL << ECCRST);
		writel (x, NAND_BASE+NFCONTROL);
		break;

	case NAND_ECC_READSYN:
		printk (KERN_DEBUG "!!! lf1000_enable_hwecc(NAND_ECC_READSYN)\n");
		/* wait for ECC decoder to be done */
		while(IS_CLR(readl(NAND_BASE+NFECCSTATUS), NFECCDECDONE));

		/* Pull out the 4 syndrome words */
		x = readl(NAND_BASE+NFSYNDRONE75);
		s7 = (x >> SYNDROM7) & 0x1fff;
		s5 = (x >> SYNDROM5) & 0x1fff;
		x = readl(NAND_BASE+NFSYNDRONE31);
		s3 = (x >> SYNDROM3) & 0x1fff;
		s1 = (x >> SYNDROM1) & 0x1fff;
		printk (KERN_DEBUG "!!!! s1=%04x s3=%04x s5=%04x s7=%04x\n", s1, s3, s5, s7);
		break;
	}
}

/* 
 * Retrieve the ECC.  Since we have Syndrome mode, this function is called only
 * on writes.  Here we need to make sure that the ECC Encoder is done encoding,
 * and then retrieve the encoded ECC so that it can be written to the OOB area.
 */

/* FYI: *dat is a 256 byte block of raw data */
int lf1000_calculate_ecc(struct mtd_info *mtd, const uint8_t *dat,
				uint8_t *ecc_code)
{
	int i = 0;
	u32 ecch, eccl;

	/* u32 x = readl(NAND_BASE+NFCNT);
	   printk (KERN_DEBUG "    NFCNT = %08x\n", x); */

	/* wait for ECC encoder to be done */
	while(IS_CLR(readl(NAND_BASE+NFECCSTATUS), NFECCENCDONE));	/* Was readw */

	ecch = readl(NAND_BASE+NFECCH) & 0xffffff;
	eccl = readl(NAND_BASE+NFECCL);

	/* retrieve encoded ECC, to be written to the OOB area */
	/* Order of ecc_code 0=low[lsb]..3=low[msb],4=high[lsb]..7=high[msb] */
	for(; i < 4; i++)
	{
		ecc_code[i] = eccl & 0xff;
		eccl >>= 8;
	}
	for(; i < 7; i++)
	{
		ecc_code[i] = ecch & 0xff;
		ecch >>= 8;
	}
	printk (KERN_DEBUG "!!!! lf1000_calculate_ecc (on write) '%10.10s' %02x %02x %02x %02x %02x %02x %02x\n", (char *)dat,
		ecc_code[0], ecc_code[1], ecc_code[2], ecc_code[3], ecc_code[4], ecc_code[5], ecc_code[6]);

	return 0;
}

/* This function is called on a read, after retrieving data and decoding the
 * ECC.  The hardware will generate an error bit and the Odd Syndrome.
 * Return: Number of corrected errors or -1 for failure */
int lf1000_correct_ecc(struct mtd_info *mtd, uint8_t *dat,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	int ret = 0;
	u32 status = readl(NAND_BASE+NFECCSTATUS);
	u32 x;
	u16 s7, s5, s3, s1;
	int lf1000_GetErrorLocation (int *pLocation, u16 s1, u16 s3, u16 s5, u16 s7);
	int loc, numFixed;

	printk (KERN_DEBUG "!!! lf1000_correct_ecc\n");

	/* Check if hardware detected any error at all */
	if(IS_SET(status,NFCHECKERROR)) {
		/* Erorr detected */
		printk("!!!! mtd-lf1000: ECC reports an error\n");

		/* TODO: Here we need to use the Odd Syndrome to determine the
		 *       error position, and then try to fix it.
		 */
		
		/* Pull out the 4 syndrome words */
		x = readl(NAND_BASE+NFSYNDRONE75);
		s7 = (x >> SYNDROM7) & 0x1fff;
		s5 = (x >> SYNDROM5) & 0x1fff;
		x = readl(NAND_BASE+NFSYNDRONE31);
		s3 = (x >> SYNDROM3) & 0x1fff;
		s1 = (x >> SYNDROM1) & 0x1fff;
		numFixed = lf1000_GetErrorLocation (&loc, s1, s3, s5, s7);
		printk (KERN_DEBUG "!!!! Correct: s1=%04x s3=%04x s5=%04x s7=%04x test=%d Loc=%d\n", s1, s3, s5, s7, numFixed, loc);
		if (numFixed < 0) {
			/* We got more than one problem */
		}
		else {
			/* Exactly one bit can be fixed, it is bit loc, counting 0=lsb of 1st byte, 8=lsb of 2nd byte, etc. */
		}
		ret = numFixed;
	}
	return ret;
}

/******************************************************************************
 * Lifted from Magic Eyes ./vtk/test/NAND_Flash/mes_nand.c
 *
//  Copyright (C) 2007 MagicEyes Digital Co., All Rights Reserved
//  MagicEyes Digital Co. Proprietary & Confidential
//
//	MAGICEYES INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
//  AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
//  FOR A PARTICULAR PURPOSE.
 *
 * MES_NAND_GetErrorLocation( int *pLocation )
 ******************************************************************************/

extern const short BCH_AlphaToTable[8192];
extern const short BCH_IndexOfTable[8192];
extern const unsigned int BCH_TGRTable[52][2];

int lf1000_GetErrorLocation (int *pLocation, u16 s1, u16 s3, u16 s5, u16 s7)
{
	register int i, j, elp_sum ;
	int count;
	int r;			// Iteration steps
	int Delta; 		// Discrepancy value
	int elp[8+1][8+2]; 	// Error locator polynomial (ELP)
	int L[8+1];		// Degree of ELP
	int B[8+1][8+2];	// Scratch polynomial
	//	int root[8+1];	// Roots of ELP
	int reg[8+1];		// Register state
	int	s[8];

	s[0] = s1;
	s[2] = s3;
	s[4] = s5;
	s[6] = s7;

	// Even syndrome = (Odd syndrome) ** 2
	for( i=1,j=0 ; i<8 ; i+=2, j++ )
	{
		if( s[j] == 0 )		s[i] = 0;
		else				s[i] = BCH_AlphaToTable[(2 * BCH_IndexOfTable[s[j]]) % 8191];
	}

	// Initialization of elp, B and register
	for( i=0 ; i<=8 ; i++ )
	{
		L[i] = 0 ;
		for( j=0 ; j<=8 ; j++ )
		{
			elp[i][j] = 0 ;
			B[i][j] = 0 ;
		}
	}

	for( i=0 ; i<=4 ; i++ )
	{
		//		root[i] = 0 ;
		reg[i] = 0 ;
	}

	elp[1][0] = 1 ;
	elp[1][1] = s[0] ;

	L[1] = 1 ;
	if( s[0] != 0 )
		B[1][0] = BCH_AlphaToTable[(8191 - BCH_IndexOfTable[s[0]]) % 8191];
	else
		B[1][0] = 0;

	for( r=3 ; r<=8-1 ; r=r+2 )
	{
		// Compute discrepancy
		Delta = s[r-1] ;
		for( i=1 ; i<=L[r-2] ; i++ )
		{
			if( (s[r-i-1] != 0) && (elp[r-2][i] != 0) )
				Delta ^= BCH_AlphaToTable[(BCH_IndexOfTable[s[r-i-1]] + BCH_IndexOfTable[elp[r-2][i]]) % 8191];
		}

		if( Delta == 0 )
		{
			L[r] = L[r-2] ;
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				elp[r][i] = elp[r-2][i];
				B[r][i+2] = B[r-2][i] ;
			}
		}
		else
		{
			// Form new error locator polynomial
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				elp[r][i] = elp[r-2][i] ;
			}

			for( i=0 ; i<=L[r-2] ; i++ )
			{
				if( B[r-2][i] != 0 )
					elp[r][i+2] ^= BCH_AlphaToTable[(BCH_IndexOfTable[Delta] + BCH_IndexOfTable[B[r-2][i]]) % 8191];
			}

			// Form new scratch polynomial and register length
			if( 2 * L[r-2] >= r )
			{
				L[r] = L[r-2] ;
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					B[r][i+2] = B[r-2][i];
				}
			}
			else
			{
				L[r] = r - L[r-2];
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					if( elp[r-2][i] != 0 )
						B[r][i] = BCH_AlphaToTable[(BCH_IndexOfTable[elp[r-2][i]] + 8191 - BCH_IndexOfTable[Delta]) % 8191];
					else
						B[r][i] = 0;
				}
			}
		}
	}

	if( L[8-1] > 4 )
	{
		//return L[8-1];
		return -1;
	}
	else
	{
		// Chien's search to find roots of the error location polynomial
		// Ref: L&C pp.216, Fig.6.1
		for( i=1 ; i<=L[8-1] ; i++ )
			reg[i] = elp[8-1][i];

		count = 0;
		for( i=1 ; i<=8191 ; i++ )
		{
			elp_sum = 1;
			for( j=1 ; j<=L[8-1] ; j++ )
			{
				if( reg[j] != 0 )
				{
					reg[j] = BCH_AlphaToTable[(BCH_IndexOfTable[reg[j]] + j) % 8191] ;
					elp_sum ^= reg[j] ;
				}
			}

			if( !elp_sum )		// store root and error location number indices
			{
				//				root[count] = i;

				// Convert error location from systematic form to storage form
				pLocation[count] = 8191 - i;

				if (pLocation[count] >= 52)
				{
					// Data Bit Error
					pLocation[count] = pLocation[count] - 52;
					pLocation[count] = 4095 - pLocation[count];
				}
				else
				{
					// ECC Error
					pLocation[count] = pLocation[count] + 4096;
				}

				if( pLocation[count] < 0 )	return -1;
				/*
				  if( i <= 8191 - 52 )	pLocation[count] = 4095 - (8191 - 52 - i);
				  else					pLocation[count] = 4095 - (8191 + 4096 - i);
				*/

				count++;
			}
		}

		if( count == L[8-1] )	// Number of roots = degree of elp hence <= 4 errors
		{
			return L[8-1];
		}
		else	// Number of roots != degree of ELP => >4 errors and cannot solve
		{
			return -1;
		}
	}
}

#endif /* CPU_LF1000 && CONFIG_MTD_NAND_LF1000_HWECC */
