/*
 * Copyright 2010, 2011 Michael Ossmann
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "inttypes.h"

#define NUM_CHANNELS 79
#define BT_NUM_CHANNELS 79
#define SEQUENCE_LENGTH 134217728
#define CHANNEL (uint8_t) 39
//#define CHANNEL (uint8_t) 71

/////////////////////////////////////////////////////////////////////////////////
/// Global Zoo
#define MAX_PKTS_IN_FILE 30000
//#define MAX_PKTS_IN_FILE1 30
#define MAX_LIST_SIZE 2048
//int l3=0, l4=0, l5=0, err_count = 0;
int global_no_winners = 0;
uint32_t candd1=0, jj=0 ;//, list44 [MAX_PKTS_IN_FILE1],  search_list [MAX_LIST_SIZE];
//uint8_t CLK6_1 [MAX_PKTS_IN_FILE1];//, search_CLK6_1[MAX_LIST_SIZE];
uint8_t found_cand = 0, stop_ubertooth = 0;


////////////////////////////////////////////////////////////
typedef struct {

	/* these values for hop() can be precalculated in part (e.g. a1 is the
	 * precalculated part of a) */
	int a1, b, c1, d1, e, fst_pkt_time;
	uint8_t AFH;
	uint32_t master_MAC;
	uint32_t LAP;
	uint32_t UAP;
	uint32_t UAP_LAP;
	uint64_t syncword;

	// GT seq length = 134217728
	uint8_t GT_seq[SEQUENCE_LENGTH];
	uint8_t bank[NUM_CHANNELS];
	//134217728/64=2097152

//	uint32_t CLK_cand2 [SEQUENCE_LENGTH/64];
	uint32_t CLK_candinc [SEQUENCE_LENGTH/64];
	uint32_t cand_hamm [SEQUENCE_LENGTH/64];
	//number of candidates
	int num_hamm_cand;
//	int num_cand2;
	int num_candinc;
	int GT_len;
	// We assume 1000 pkts are initially observed, more pkts can be considered
//	int dist [1000];

} pico;



//////////////////////////////////////////////////////////////////////////////
struct _ShMemory {

////////// CLK acq
	uint8_t 	OneCh_status;
//	uint8_t 	OneCh_clk6_1     	[ MAX_PKTS_IN_FILE ];
//	int 		OneCh_slts 	 	[ MAX_PKTS_IN_FILE ];

	uint8_t 	* OneCh_clk6_1     	;
	int 		* OneCh_slts 		;

	int 		OneCh_npkts;


	uint32_t 	wCand ;
	uint32_t 	wCand_idx_jj ;

};
//////////////////////////////////////////////
void init_shmem  ( struct _ShMemory *shm )
{

	shm->OneCh_clk6_1 		= (uint8_t  *) malloc ( MAX_PKTS_IN_FILE );
	shm->OneCh_slts 		= (int      *) malloc ( MAX_PKTS_IN_FILE );

}
/////////////////////////////////////////////////////
void deinit_shmem ( struct _ShMemory *shm )
{
	free (shm->OneCh_clk6_1	  );
	free (shm->OneCh_slts	  );
}

//////////////////////////////////////////////////////////////////////////////////////////
/* 5 bit permutation */
/* assumes z is constrained to 5 bits, p_high to 5 bits, p_low to 9 bits */
int perm5(int z, int p_high, int p_low)
{
	int i, tmp, output, z_bit[5], p[14];
	int index1[] = {0, 2, 1, 3, 0, 1, 0, 3, 1, 0, 2, 1, 0, 1};
	int index2[] = {1, 3, 2, 4, 4, 3, 2, 4, 4, 3, 4, 3, 3, 2};

	/* bits of p_low and p_high are control signals */
	for (i = 0; i < 9; i++)
		p[i] = (p_low >> i) & 0x01;
	for (i = 0; i < 5; i++)
		p[i+9] = (p_high >> i) & 0x01;

	/* bit swapping will be easier with an array of bits */
	for (i = 0; i < 5; i++)
		z_bit[i] = (z >> i) & 0x01;

	/* butterfly operations */
	for (i = 13; i >= 0; i--) {
		/* swap bits according to index arrays if control signal tells us to */
		if (p[i]) {
			tmp = z_bit[index1[i]];
			z_bit[index1[i]] = z_bit[index2[i]];
			z_bit[index2[i]] = tmp;
		}
	}

	/* reconstruct output from rearranged bits */
	output = 0;
	for (i = 0; i < 5; i++)
		output += z_bit[i] << i;

	return(output);
}

/* generate the complete hopping sequence */
void gen_hops(pico *pn)
{
	/* a, b, c, d, e, f, x, y1, y2 are variable names used in section 2.6 of the spec */
	/* b is already defined */
	/* e is already defined */
	int a, c, d, f, x;
	int h, i, j, k, c_flipped, perm_in, perm_out;

	/* sequence index = clock >> 1 */
	/* (hops only happen at every other clock value) */
	int index = 0;
	f = 0;

	/* nested loops for optimization (not recalculating every variable with every clock tick) */
	for (h = 0; h < 0x04; h++) { /* clock bits 26-27 */
		for (i = 0; i < 0x20; i++) { /* clock bits 21-25 */
			a = pn->a1 ^ i;
			for (j = 0; j < 0x20; j++) { /* clock bits 16-20 */
				c = pn->c1 ^ j;
				c_flipped = c ^ 0x1f;
				for (k = 0; k < 0x200; k++) { /* clock bits 7-15 */
					d = pn->d1 ^ k;
					for (x = 0; x < 0x20; x++) { /* clock bits 2-6 */
						perm_in = ((x + a) % 32) ^ pn->b;
						/* y1 (clock bit 1) = 0, y2 = 0 */
						perm_out = perm5(perm_in, c, d);
						pn->GT_seq[index] = pn->bank[(perm_out + pn->e + f) % BT_NUM_CHANNELS];
						if (pn->AFH) {
							pn->GT_seq[index + 1] = pn->GT_seq[index];
						} else {
							/* y1 (clock bit 1) = 1, y2 = 32 */
							perm_out = perm5(perm_in, c_flipped, d);
							pn->GT_seq[index + 1] = pn->bank[(perm_out + pn->e + f + 32) % BT_NUM_CHANNELS];
						}
						index += 2;
					}
					f += 16;
				}
			}
		}
	}
}
////////////////////////////////////

void address_precalc(int32_t address,  pico *pn)
{
	int i;
	/* populate frequency register bank*/
	for (i = 0; i < BT_NUM_CHANNELS; i++)
			pn->bank[i] = ((i * 2) % BT_NUM_CHANNELS);
	/* actual frequency is 2402 + pn->bank[i] MHz */

	/* precalculate some of single_hop()/gen_hop()'s variables */
	pn->a1 = (address >> 23) & 0x1f;
	pn->b = (address >> 19) & 0x0f;
	pn->c1 = ((address >> 4) & 0x10) +
		((address >> 3) & 0x08) +
		((address >> 2) & 0x04) +
		((address >> 1) & 0x02) +
		(address & 0x01);
	pn->d1 = (address >> 10) & 0x1ff;
	pn->e = ((address >> 7) & 0x40) +
		((address >> 6) & 0x20) +
		((address >> 5) & 0x10) +
		((address >> 4) & 0x08) +
		((address >> 3) & 0x04) +
		((address >> 2) & 0x02) +
		((address >> 1) & 0x01);
}
////////////////////////////////////////////////////////
// use GT_seq to find out CLK candidates
void init_candidates(pico *pn, uint8_t ch)
{
	int i, k=0; 
	/* populate frequency register bank*/

	for (i = 0; i < SEQUENCE_LENGTH ; i++){

		if (ch == pn->GT_seq[i]){
			pn->CLK_candinc [k  ] = i;
			pn->cand_hamm   [k  ] = 0;			
			k++;	  
		}
	}
	/* actual frequency is 2402 + pn->bank[i] MHz */
	pn->num_candinc    = k;
	pn->num_hamm_cand  = k;

}

///////////////////////////////////////////////////////////////////////////////////
static uint32_t hamm_ds5_inc0722 (pico * pn1, FILE * fout, const struct _ShMemory *shm, int l, uint8_t CLK6_1, const int start_from )
{
//dts      = is a list of distances (# of slots) between consecutive pkts of the same class
// l       = is the length of CLK6_1
// CLK_candinc [j] = CLK candidates extracted from GT assuming ubertooth listens to ch=39
// num_candinc = # of CLK candidates
	int i,j, cand_hamm2, max_hamm2=0;
	uint32_t cand, dst_to_fst=0;
	for (j = 0; j < pn1->num_candinc - l; j++ )
	{

		dst_to_fst=0; cand_hamm2=0; 

		// take one cand atime
		cand = pn1->CLK_candinc [j]; 

		// for each incoming candidate cand find the distance between cand and the list
		for (i=0 ; i < l; i++)
		{
//			dst_to_fst = dst_to_fst + dst[i];
			dst_to_fst = dst_to_fst + shm->OneCh_slts [ i + start_from ];

			if ( CHANNEL == pn1->GT_seq [ ( cand + dst_to_fst ) % SEQUENCE_LENGTH  ] )
				{cand_hamm2++; }

		}


		if ( (l/4) <= cand_hamm2)  
		{
			if ( (63 & cand) == CLK6_1)
			{
				if (max_hamm2 <= cand_hamm2)
				{
					max_hamm2 = cand_hamm2; 
					candd1 = cand ;
				}

				fprintf (fout, "%d, %u, %u\n",              cand_hamm2, cand, candd1);
				++ found_cand ;
			}

		}

	}

out:
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////

//uint32_t address = 0x2a96ef25;
//uint32_t address = 0xdc065d23;
//uint32_t address = 0xdc06c0b3;
//uint32_t address = 0xdc0662ef;
//uint32_t address = 0x72c6653b;
//uint32_t address = 0x723397d3;
//uint32_t address = 0x72339685;
//uint32_t address = 0x72c61600;
//uint32_t address = 0xF889A175;
//uint32_t address = 0x72c66a0a;
//uint32_t address = 0x723397d3;
//uint32_t address = 0x01F3E10A;
//uint32_t address = 0xDC065D7F;
//uint32_t address = 0x72C62E8A;
//uint32_t address = 0x6AFE2C6F;
//uint32_t address = 0x72C66A2D;
//uint32_t address = 0xACE857F0;
//uint32_t address = 0xDC0662EF;
//uint32_t address = 0xDC065D7F;
uint32_t address = 0xDC065D58;
//uint32_t address = 0x72C669FD;
//uint32_t address = 0x72C62E7E;
//uint32_t address = 0x7223B0DD;
//uint32_t address = 0x72C669E2;
//uint32_t address = 0x6AFE2C6F;
//uint32_t address = 0x72C66A1C;
//uint32_t address = 0x72C66A0E;
//uint32_t address = 0x6AFE334B;
//uint32_t address = 0x6AFE2F40;
//uint32_t address = 0x6AFE2C6F;
//uint32_t address = 0x72C66A54;
//uint32_t address = 0xDD6C88A3;// Mouse
//uint32_t address = 0x72C61604;
//uint32_t target_LAP;
// 32 bits of the MAC, i.e. LAP + UAP

void init_piconet(pico *pn1, const uint32_t address)
{

	// We are working on AFH79
	pn1->AFH=1;
	// We are working on Basic nonAFH
//	pn1->AFH=0;  
	pn1->fst_pkt_time = 0;
	pn1->master_MAC = address;
	pn1->LAP = address & 0xffffff;
	pn1->UAP = (address >> 24) & 0xff;

	pn1->syncword  = 0 ; //btbb_gen_syncword(pn1->LAP);

	// Do precalculation
	address_precalc(address, pn1);

	//Generate the ground truth seque
	gen_hops(pn1);

	// Generate a list of CLK cand, specify a channel CHANNEL = 39
	init_candidates(pn1, CHANNEL);


}
//////////////////////////////////////////////////////////////////////////////
int round_slots(float x)
{
	// Threshold here is 0.1
//	if ( (int) (x) < (int) (x+0.1))
//	if ( (int) (x) < (int) (x+0.3))
	if ( (int) (x) < (int) (x+0.5))
		return (int) (x+1);
	else 
		return (int) x;
}

/////////////////////////////////////////////////////////////////////////////////
int read_pkts_from_file2 ( struct _ShMemory *ShmPTR, const int max_pkts_to_read )

{

	FILE * pFile, *fout;
	int round_slts=0, n_pkts ;
	float slots;
	uint32_t pkt_time, pkt_LAP, prev_pkt_time = 0xffffffff;
	uint8_t pkt_clk6_1, pkt_type, pkt_LT_ADD, pkt_rx_channel, prev_clk6_1 = 0;

//	pFile = fopen ("tafh79_111","r");
//	pFile = fopen ("tafh79_ch39_1","r");
//	pFile = fopen ("new_afh79","r");
//	pFile = fopen ("tdata_afh79_ch39_1","r");
	pFile = fopen ("tdata_afh79_ch39_3","r");
//	pFile = fopen ("tsound_jam20_ch39_1","r");
//	pFile = fopen ("tsound_jam40_ch39_1","r");
	if (NULL == pFile) { printf ("Err open file"); goto out; }

	n_pkts = 0 ;
// Read pkt loop
	while ( 1 )
	{


		fscanf (pFile, "%"SCNu8", %"SCNu8", %"SCNu8", %"SCNu8", %6x, %u, %d, %f", 
		&pkt_clk6_1, 
		&pkt_type, 
		&pkt_LT_ADD, 
		&pkt_rx_channel, 
		&pkt_LAP, 
		&pkt_time, 
		&round_slts,
		&slots
		);

		if (feof(pFile)) break;

//		if ( MAX_PKTS_IN_FILE <= n_pkts) break;
		if ( max_pkts_to_read <= n_pkts) break;

//		printf ("%"SCNu8", %"SCNu8", %"SCNu8", %"SCNu8", %6x, %u, %d, %f\n", 
//		pkt_clk6_1, 
//		pkt_type, 
//		pkt_LT_ADD, 
//		pkt_rx_channel, 
//		pkt_LAP, 
//		pkt_time, 
//		round_slts,
//		slots
//		);


		ShmPTR->OneCh_slts 	[ n_pkts ] = round_slts;
		ShmPTR->OneCh_clk6_1    [ n_pkts ] = pkt_clk6_1;

		++ n_pkts;
//	if ( 5 < n_pkts) 
//		break ;
	}


	ShmPTR->OneCh_npkts = n_pkts ;

	fclose (pFile);
	return n_pkts;

out:
	return 0;

}

/////////////////////////////////////////////////////////////////////////
int clock_search2 (pico * pn1, struct _ShMemory *ShmPTR, int lower_indx, int upper_indx, int search_list_length, int scanned_pkts )
{

	FILE * fout;
	int info = -1, no_slots=0, i, j;
//	int search_list1 [ MAX_LIST_SIZE ] ;

	char fname [1024];
	sprintf (fname, "f%d", search_list_length);

	fout = fopen (fname, "w");
	if (NULL == fout) { printf ("Err open file"); goto out; }
	int k = 0;
//	for (i = 0 ; i < 100 ; i ++)
//	for (i = 0 ; i < n_pkts1  ; i ++)
//	for (i = 0 ; k < n_pkts1  && i < scanned_pkts; i ++)
	for (i = lower_indx ; (i < upper_indx)  &&  (i < scanned_pkts); i ++)
	{

//		for (j = 0; j < search_list_length; j++)
//			search_list1 [j] = ShmPTR->OneCh_slts [j + i + 1];

		if ( -1 == info) 
		{
			// test list length = 100, total captured pkts = x
			for ( j = 0 ; j < search_list_length ; j++ )
//				no_slots += search_list1 [j];
				no_slots += ShmPTR->OneCh_slts [ j + i + 1 ] ;

			fprintf (fout, "777777, %d, %d\n", search_list_length, upper_indx  );
//			fprintf (fout, "777777, %d, %d\n", search_list_length, lower_indx  );
			info = 1;
			no_slots = 0;
		}	
		if ( 1 == info) 
		{

//			hamm_ds5_inc0722(pn1, fout, search_list1, search_list_length, ShmPTR->OneCh_clk6_1 [ i ] );
			hamm_ds5_inc0722(pn1, fout, ShmPTR      , search_list_length, ShmPTR->OneCh_clk6_1 [ i ] , i + 1 );

			for ( j = 0 ; j < search_list_length ; j ++ )
//				no_slots += search_list1 [j];
				no_slots += ShmPTR->OneCh_slts [ j + i + 1 ] ;
// this line was for test
//			fprintf (fout, "999999, %d, %d, %u, %u, %u\n", no_slots, search_list[0], candd1, candd1 & 0x3f, CLK6_1[i]);
//			fprintf (fout, "999999, %d, %d\n", no_slots, search_list1[0]);
			fprintf (fout, "999999, %d, %d\n", no_slots, ShmPTR->OneCh_slts [ i + 1 ] );
			no_slots = 0;


		}

		if ( 1 == stop_ubertooth)
		{
			break ;

		}

	}

out:
	fclose (fout);
	return 0;
}

/////////////////////////////////////////////////////////////////////
// For statistic you need to specify a ground truth clk
const uint32_t ground_truth_clk = 109114503; 
void do_statistics2(int list_length)
{
	FILE * pFile;
	int i, h2, curr_cand, cand, no_slots=0, avg_no_slots=0, total_no_pkts=0, n_pkts=0, search_list_length, pkt_x, candidates [1024];

	char fname [1024];
	sprintf (fname, "f%d", list_length);

	pFile = fopen (fname,"r");

	if (pFile==NULL) { printf ("Err open file"); goto out; }


	fscanf (pFile,"%d, %d, %d,", &h2, &search_list_length, &total_no_pkts);

	if ( 777777 != h2) // final data file should start with 777777
		{ printf ("no 77777\n"); goto out;}

int non_winners = 0, winners = 0, failures = 0;
float CDF, avg_slots;

	while ( 1 )
	{

		fscanf (pFile,"%d, %d, %d", &h2, &curr_cand, &pkt_x);

		if (feof(pFile)) { break; }

		if ( 999999 == h2)
		{

			if ( 1  < n_pkts )
			{
				++ non_winners; // we have several cands
			}

			if ( 1 == n_pkts ) // we have a unique winner
			{
				printf ("%d, %d\n",candidates[0] , ground_truth_clk + no_slots);
				if ( candidates[0] == ( (int)ground_truth_clk + no_slots) )
				++ winners ; 
				avg_no_slots += curr_cand;

			}
//			else 
			if ( 0 == n_pkts )
			{
				++ failures; // no cand at all

			}

			no_slots += pkt_x;
			n_pkts = 0;
		}
		else 
		{
			candidates [n_pkts] = curr_cand;
			++ n_pkts ;

		}

	}

out:
//	printf ("winners=%d, non_winners=%d, avg_slts=%f, total_no_pkts=%d\n", winners, non_winners, 0.000625 * (float) no_slots/total_no_pkts, total_no_pkts -non_winners );
// 
avg_slots 	= (float) avg_no_slots / winners;
CDF 		= (float) winners / (total_no_pkts - non_winners) ;
//	printf ("%d, %04d, %03d, %.03f, %.03f, %.03f, %d\n", 
//	search_list_length, 
//	winners, 
//	non_winners,
//	CDF,
//	avg_slots,
//	0.000625 * avg_slots,
//	total_no_pkts -non_winners 
//	);
	printf ("%d, %03d, %03d, %03d, %.03f, %d\n", 
	search_list_length, 
	winners, 
	non_winners,
	failures,
//	CDF,
	avg_slots,
//	0.000625 * avg_slots,
	winners + non_winners + failures
	);

	global_no_winners = winners;

	fclose (pFile);

}
//////////////////////////////////////////////////////////////////////////////
void cleanup(int sig)
{
	sig = sig;
	stop_ubertooth = 1;

}
///////////////////////////////////////////////////////
int main ()
{
//	pico * pn1 = NULL;
	pico *pn = malloc(sizeof *pn);
	init_piconet (pn, address );
//	pn1 = pn;

	int search_length = 2, scanned_pkts = 0, lower_indx = 2000, upper_indx = 3000, max_pkts_to_read = 6000 ;

	// These are lower_cand and upper_cand ; 
	// For all data files and voice files the starting slots no is 366384
//	uint32_t lower_candd  = ground_truth_clk= 366313, upper_candd = 1000000;//,  ground_truth_clk = lower_candd;
//	uint32_t lower_candd  = 0, upper_candd = 400;//,  ground_truth_clk = lower_candd;
//	int no_slots = (int) (upper_candd - lower_candd);
		
	struct _ShMemory ShMem;
	init_shmem  ( &ShMem );
	scanned_pkts = read_pkts_from_file2 ( &ShMem , max_pkts_to_read + 1000 );
//	scanned_pkts = read_pkts_from_file1 (        );
	printf ("scanned_pkts = %d\n", scanned_pkts);


	/* Clean up on exit. */
	signal(SIGINT,cleanup);
	signal(SIGQUIT,cleanup);
	signal(SIGTERM,cleanup);



	while ( search_length < 1060 )
//	while ( search_length < 200 )
	{
		if (scanned_pkts < (search_length + upper_indx ) ) 
		{
			printf ("Err upper_limit\n"); 
			break ; //goto out;
		}

		clock_search2    ( pn, &ShMem, lower_indx, upper_indx, search_length, scanned_pkts);


//		do_statistics2  ( search_length );

		search_length += 50;

		if ( 1 == stop_ubertooth)
		{
			break ;

		}

	}

out:
	free (pn);
	deinit_shmem ( &ShMem );
	return 0;

}

