/*
 * Copyright (c) 2015, Computer Science Department (CSD), University of Crete, and 
 * Signal Processing Lab (SPL), Institute of Computer Science (ICS), FORTH, Greece.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CSD, SPL/ICS-FORTH nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * 
 */

/**
 * \file
 *  The implementation of Local Minimum spanning Tree for topology control in contiki. The implementation is based at the
 *  description given at: N. Li, J. Hou, and L. Sha, “Design and analysis of an mst-based topology control algorithm,” Wireless Communications, IEEE Transactions on, vol. 4, no. 3, pp. 1195–1206, May 2005.
 *
 *
 *
 * \author
 *      Phivos Phivou <fivou@csd.uoc.gr>, Nancy Panousopoulou - <apanouso@ics.forth.gr>
 */

#include <stdio.h>
#include "contiki.h"
#include "cc2420.h"
#include "net/rime.h"
#include "net/packetbuf.h"
#include "dev/serial-line.h"
#include "random.h"
#include <string.h>


#include <stdlib.h> 


#include "lib/list.h"
#include "lib/memb.h"
#include <math.h>

#define TIMEFRAME 30
#define RED_WAIT 30

#define MAXVAL 10000 //for Prim algorithm

#define PRES_CONST 200000u // for conversion from float to int
#define PRES_RANGE 1000u // likewise

#define MAX_NEIGHBORS 20 //maximum number of neighbors

#define SEND_TIME (random_rand() % (4*CLOCK_SECOND))

//for 15m tx range.
static const
float POWER [8] = {
  2.107  ,
  11.472 ,
  28.330 ,
  52.680 ,
  84.520 ,
  123.850,
  170.680,
  225.000
};
/* This structure holds information about 1-hop neighbors. */
struct neighbor {
  /* The ->next pointer is needed since we are placing these on a
     Contiki list. */
  struct neighbor *next;

  /*current node, 1-hop neighbour */
  uint16_t node_i;
  uint16_t node_j;
  /*weight: distance*/
  float weight;
};
 
/*for conversions from float to char[4] and vise versa*/
union ff{
  float f;
  unsigned char b4[sizeof(float)];
};

/*this structure holds information about 2-hop neighbors.*/

struct twohopneighs {
  struct twohopneighs *next;
  uint16_t id;
  uint16_t * nn;
  float * w;
  uint8_t len;
};

//EDGES for MST calculation
struct edge {
  struct edge *next;
  uint8_t i_ind;
  uint8_t * j_ind;
};


	
	
static float myx,myy;
static uint16_t numofv;
static uint8_t parent[MAX_NEIGHBORS+1]; 
static process_event_t lmst_start_event, lmst_finish_event, prim_start_event, prim_finish_event;
float str2float(char *str);




MEMB(neighbor_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbor_list);

MEMB(twohopneighs_memb, struct twohopneighs,MAX_NEIGHBORS);
LIST(twohopneighs_list);


MEMB(edge_memb, struct edge, MAX_NEIGHBORS);
LIST(edge_list);

/*---------------------------------------------------------------------------*/
PROCESS(b_process, "Broadcast thread");
PROCESS(lmst_process, "LMST thread");
PROCESS(prim_process, "PRIM");
AUTOSTART_PROCESSES(&b_process, &lmst_process,&prim_process);
/*---------------------------------------------------------------------------*/

//removing 1- and 2-hop neighbors from respective lists.
static char
remove_2hopneighbor(void *n)
{
  struct twohopneighs *e = n;

  list_remove(twohopneighs_list, e);
  return memb_free(&twohopneighs_memb, e);
 
}

static char
remove_neighbor(void *n)
{
  struct neighbor *e = n;

  list_remove(neighbor_list, e);
  return memb_free(&neighbor_memb, e);
  
  
}

//----------------------------------------------------------------------
float str2float(char * str)
{
  float r, num;
  char * s = strdup(str);
  char * a =(char *) strtok(s, "."), b[3];
  num = atoi(a);
  r = num;
  a = (char *) strtok(NULL, ".");
  b[0] = a[0];
  b[1] = a[1];
  b[2] = a[2];
  num = atoi(b);
  r += (r<0 ? - (num / 1000) : (num / 1000));
  return r;
}


//----------------------------------------------------------------------
//----------------------------------------------------------------------
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  
  char * tmp1;
  char *x=0, *y=0;
  float dx, dy;
  struct neighbor *e=0;
  struct twohopneighs *ee=0;
  uint16_t ii,kk, ll, vall;
  uint8_t * tttmp;
  //union ff f1;
  
  
  //int16_t rssi = (int16_t)(packetbuf_attr(PACKETBUF_ATTR_RSSI) - 45);
  
  
  tmp1 = (char *)packetbuf_dataptr();
 
  if(tmp1[0] == 'N'){ //final - neighbourhood rediscovery
    printf("NEIGHBOR %d\n", from->u8[0]);
    /*free(x); 
    free(y);
    free(e);
    free(tmp1);
    free(ee);*/ 
    return;
  }
 
 
  if (tmp1[0] == '$') //1-hop neighbour discovery (initial step)
    {
      tmp1 = strdup((char *)&tmp1[1]);

      x = strtok(tmp1, "#");
      y = strtok((char *)NULL, "#");
 
      dx = str2float(x);
      dy = str2float(y);
      if (list_length(neighbor_list) >= MAX_NEIGHBORS){
	/*free(x);
	free(y);
	free(e);
	free(ee);
	free(tmp1);*/
	return;	  
      }
	  	  
      for(e = list_head(neighbor_list); e != NULL; e = list_item_next(e)) {
	if(from->u8[0] ==  e->node_j) {
	  /* free(x);
	     free(y);
	     free(e);
	     free(ee);
	     free(tmp1);*/
	  return;
	}
      }
 
      if (from->u8[0] == rimeaddr_node_addr.u8[0]){
	return;
      }
      //place in 1-hop neighbours list	
      e = memb_alloc(&neighbor_memb);
      if(e != NULL) {
	e->node_i = rimeaddr_node_addr.u8[0];
	e->node_j = from->u8[0];
	//set weight
	e->weight =  (float)((myy - dy) * (myy - dy) + (myx - dx) * (myx - dx));
	list_add(neighbor_list, e);
      }
      /*
      free(x); 
      free(y);
      free(e);
      free(tmp1);
      free(ee);*/
      return;


    }
  
  //if you have reached at this point, then you are in the 2nd hop neighbourhood discovery.
  if (list_length(twohopneighs_list) >= MAX_NEIGHBORS){
    /*    free(x);
    free(y);
    free(e);
    free(ee);
    free(tmp1);*/
    return;
	  
  }
  //2-hop neighbor
  for (ee = list_head(twohopneighs_list); ee !=NULL; ee = list_item_next(ee)) {
    if (from->u8[0] == ee -> id) {
      //shouldnt be in here...
      free(x); 
      free(y);
      free(e);
      free(tmp1);
      free(ee); 
      return; 
    }
	  
	  
  }
  
  //place in 2-hop neighbourhood
  ee = memb_alloc(&twohopneighs_memb);
  
  if (ee !=NULL){
    tttmp = (uint8_t *)packetbuf_dataptr();
	
    ll = packetbuf_datalen() / 3; //3 BYTES per neighbour
    ee->id = from->u8[0];
    ee->len = ll;
	  
    ee->nn = malloc(sizeof(uint16_t)*ll);
    ee->w = malloc(sizeof(float)*ll);
    for (ii=0;ii<ll;ii++){
      ee->nn[ii] =0;
      ee->w[ii] = 0.0;
    }
	  
    ii = 0;
    for (kk= 0; kk< ll*3;kk=kk+3)
      {
	ee->nn[ii] = tttmp[kk];
	vall = tttmp[kk+1]<<8 | tttmp[kk+2];


	ee->w[ii] = (float)(vall * 1000.0 / 200000.00); //you are receing in uint16 - convert back to float
	ii++;  
      }
    list_add(twohopneighs_list, ee);
  }
  
  free(x); 
  free(y);
  free(e);
  free(tmp1);
  free(ee); 
  
  
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;




/*MAIN THREAD------------------------------------------------------------------------------*/
PROCESS_THREAD(b_process, ev, data)
{
  static struct etimer send_timer, redelca_timer;
  
  static uint32_t start_energy_cpu, start_energy_rx, start_energy_tx;
  static uint32_t end_energy_cpu, end_energy_rx, end_energy_tx;
 
  static char * msg, * my_x, * my_y, * position;
  
  static struct neighbor * new;
  static uint16_t i, k,val;
  
  
  static uint8_t *msg1;  
 
  PROCESS_EXITHANDLER(broadcast_close(&broadcast));
  PROCESS_BEGIN();

  /* Initialize the memory for the neighbor table entries. */
  memb_init(&neighbor_memb);

  /* Initialize the list used for the neighbor table. */
  list_init(neighbor_list);
  
  /* Initialize the memory for the neighbor table entries. */
  memb_init(&twohopneighs_memb);

  /* Initialize the list used for the neighbor table. */
  list_init(twohopneighs_list);

  broadcast_open(&broadcast, 129, &broadcast_call);

  /*Wait for the position from Cooja script*/
  PROCESS_YIELD_UNTIL(ev == serial_line_event_message);
  msg = strdup((char*) data);
  
  position = strdup((char*) data);
  my_x = strtok(msg, "#");
  my_y = strtok(NULL, "#");
 
  if((position = malloc(strlen("$")+strlen((char *)data)+1)) != NULL){
    position[0] = '\0';   // ensures the memory is an empty string
    strcat(position,"$");
    strcat(position,(char *)data);
  }

  
  myx = str2float(my_x);
  myy = str2float(my_y);

  /*Set timers redelca timer to 60s and broadcast timer (1st hop neighbour discovery)*/
  etimer_set(&redelca_timer,RED_WAIT*CLOCK_SECOND);
  etimer_set(&send_timer,((10  + rimeaddr_node_addr.u8[0] % TIMEFRAME) * CLOCK_SECOND));
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
  /*Prepare broadcast buffer and broadcast your position*/
 
  packetbuf_clear();
  packetbuf_clear_hdr();
  packetbuf_copyfrom(position, strlen(position));
  broadcast_send(&broadcast);
	
  //free(my_x);
  //free(my_y);
 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  
  //free(msg);  
  //free(position);
  
  //-------------------2-hop neighbour discovery------------------------
  //start--------------------->>
  start_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  start_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  start_energy_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  
  if (list_length(neighbor_list) > 0) {
    i=0;

    msg1 = malloc(sizeof(uint8_t)*3*MAX_NEIGHBORS);
    //format is: node id (1B) , weight (2B)
    
    for(new = list_head(neighbor_list); new != NULL; new = list_item_next(new))
      {
	msg1[i++] =  new->node_j & 0xFF;
	val = (uint16_t)floor(new->weight * PRES_CONST / PRES_RANGE);
	msg1[i++] = val>>8;//msb
	msg1[i++] = val & 0xFF;//lsb
      }

    k = rimeaddr_node_addr.u8[0] % TIMEFRAME;
    k = k *3;
		
    etimer_set(&redelca_timer, RED_WAIT*2*CLOCK_SECOND); //wait to gather 2 hop neighborhood
    etimer_set(&send_timer, (k+10) * CLOCK_SECOND); //a delay for handling concurrent transmissions.
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    
    packetbuf_clear();
    packetbuf_clear_hdr();
	
    packetbuf_copyfrom(msg1, sizeof(uint8_t)*i);
    broadcast_send(&broadcast);
   
        
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  
  }
  //free(new);
  //free(msg1);
 
  end_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  end_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT); //timings for 2-hop neighbour discovery
 
  printf("ENERGY_TX %lu\n", end_energy_tx-start_energy_tx);
  printf("ENERGY_RX %lu\n", end_energy_rx-start_energy_rx);
  //<<-------------------------------------------------end

  etimer_set(&send_timer, SEND_TIME);
 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
 
  /**---------Switch control to LMST thread---------------------------**/ 
  process_post(&lmst_process, lmst_start_event,NULL);
 
  PROCESS_YIELD_UNTIL(ev == lmst_finish_event);
 
  
  end_energy_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  
  
  /**--------final 1-hop neigh (after setting the transmission power)**/
  
  start_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  start_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
 
  etimer_set(&redelca_timer,  RED_WAIT*CLOCK_SECOND);
  
  etimer_set(&send_timer, ((10 + rimeaddr_node_addr.u8[0] % TIMEFRAME) * CLOCK_SECOND));
  
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
  //Prepare broadcast buffer and broadcast message
  packetbuf_clear();
  position = strdup("N\0");
  packetbuf_attr_clear();
  packetbuf_copyfrom(position, strlen(position));
  broadcast_send(&broadcast);
  // 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  end_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  end_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  printf("ENERGY_CPU %lu\n", end_energy_cpu-start_energy_cpu);
  printf("ENERGY_TX %lu\n", end_energy_tx-start_energy_tx);
  printf("ENERGY_RX %lu\n", end_energy_rx-start_energy_rx);

 
  printf("DONE\n");
  PROCESS_END();
}

//--------------------------LMST calculation--------------------------//

PROCESS_THREAD(lmst_process, ev,data)
{

  static uint16_t ii,jj,kk,ll,maxn;

  static struct neighbor *tmpn;

  static struct twohopneighs *tmpnn;

  static struct edge *tmpe;

  static float a, maxp, b;
    
  static int mem;

  static uint8_t found;
	

  PROCESS_BEGIN();
    
  memb_init(&edge_memb);
    
  list_init(edge_list);
    
  PROCESS_YIELD_UNTIL(ev == lmst_start_event);

  printf("DELAUNAY\n");
   
  numofv = list_length(neighbor_list);
  maxn = list_length(twohopneighs_list);
 
 
  printf("NEIGHBOR %d\n",numofv);
  
  if (numofv > MAX_NEIGHBORS || maxn > MAX_NEIGHBORS){
	  
    printf("INITIALIZE %d", -255); //error handling
    goto myexit;
  }
  
 
 
  //check if in 2-hop neighborhood there are unknown 1st hop neighs
  tmpnn = list_head(twohopneighs_list);
  while (tmpnn !=NULL){
	
    found = 0;
    for (tmpn = list_head(neighbor_list); tmpn !=NULL; tmpn = list_item_next(tmpn)){

      if (tmpnn->id == tmpn->node_j)
	{
	  found=1;
	}
    }
	
    if (found ==0){
			
      if (remove_2hopneighbor(tmpnn)<0){
	printf("INITIALIZE %d\n",-5); //error handling
	goto myexit;
      }
      tmpnn = list_item_next(tmpnn);
    }
    else {
      tmpnn = list_item_next(tmpnn);
    }
  }
  numofv = list_length(neighbor_list);
  maxn = list_length(twohopneighs_list);
	      
  
  tmpn = list_head(neighbor_list);
  while (tmpn !=NULL){
    found = 0;
    for (tmpnn = list_head(twohopneighs_list); tmpnn !=NULL; tmpnn = list_item_next(tmpnn)){
      if (tmpnn->id == tmpn->node_j)
	{
	  found=1;
	}
    }
    if (found ==0){
      if (remove_neighbor(tmpn) <0){
	printf("INITIALIZE %d\n",-6); //error handling
	goto myexit;
      }
      tmpn = list_item_next(tmpn);
    }
		   
    else {
      tmpn = list_item_next(tmpn);
    }
  }
  numofv = list_length(neighbor_list);
  maxn = list_length(twohopneighs_list);
	   
  //	}
  //done with 1- and 2-hop checks.

  if (numofv != maxn) {
    printf("INITIALIZE %d\n", -1); //error handling
    goto myexit;
  }

  //let's try to create the edges for 1- and 2-hop neighbours graph.

  tmpe = memb_alloc(&edge_memb);

  if (tmpe !=NULL){
    tmpe->i_ind = 0;
    tmpe->j_ind = malloc(sizeof(uint8_t)*(numofv+1));
    if (tmpe->j_ind == NULL){
      printf("INITIALIZE %d\n", -254); //error handling
      goto myexit;
    }
    tmpe->j_ind[0] = 0; //0x0 element
    ii=1;
    for (tmpnn = list_head(twohopneighs_list); tmpnn!=NULL && ii<numofv+1; tmpnn = list_item_next(tmpnn)){

      tmpe->j_ind[ii] = 1;
      ii++;
    }

    list_add(edge_list, tmpe);
  }

  else {
    printf("INITIALIZE %d\n", -2);//error handling
    goto myexit;
  }
  ii=1;
  for(tmpn = list_head(neighbor_list); tmpn != NULL && ii<numofv+1; tmpn = list_item_next(tmpn))
    {
      tmpe = memb_alloc(&edge_memb);
      if ( tmpe!= NULL) {
	tmpe->i_ind = ii;
	tmpe->j_ind = malloc(sizeof(uint8_t)*(numofv+1));
	if (tmpe->i_ind == 0){
	  printf("INITIALIZE %d", -2);//error handling
	  goto myexit;
	}
	for (jj=0;jj<numofv+1;jj++){
	  tmpe->j_ind[jj] = 0;
	}

	tmpe->j_ind[0] = 1; // this is the current node index
	kk=1;
	for (tmpnn = list_head(twohopneighs_list); tmpnn != NULL && kk<numofv+1; tmpnn = list_item_next(tmpnn)){
	  found = 2;
		    
	  if (tmpn->node_j != tmpnn -> id){
	    if (tmpnn->len > MAX_NEIGHBORS){
	      printf("INITIALIZE %d\n", -3);//error handling
	      goto myexit;
	    }	
	    for (jj=0;jj<tmpnn->len;jj++){
	      if (tmpnn->nn[jj] == tmpn->node_j) { //if your 1-st hop neighbour is also my 1-st hop neighbour.
		found = 0;
		break;
	      }
				 
	    }
	    tmpe->j_ind[kk] = found ==0? 1:tmpe->j_ind[kk];
	  }
	  kk++;
	}
	list_add(edge_list, tmpe); //in list we add the 2-hop edges between 1-st hop neighs of the current node.
      
      }
      
      else{
	printf("INITIALIZE %d\n", -4);//error handling
		  
	goto myexit;	
	
		  
      }
      ii++;
    } 
     

  ll = 0;
  mem = 0;
  maxn = 0;
  
  printf("INITIALIZE %d\n", numofv);

  
  if (numofv<1 || numofv>MAX_NEIGHBORS+1)
    {
	
      goto myexit;
    } 
  //calculate the MST based on Prim's algorithm. 
  process_post(&prim_process, prim_start_event, NULL);
  
  PROCESS_YIELD_UNTIL(ev == prim_finish_event);

  //and then get the lmst edges of the current node and get 
  maxp = 0.0;
  
  if (numofv > 1){
    ii=1;
    for (tmpn = list_head(neighbor_list); tmpn !=NULL && ii<numofv+1; tmpn=list_item_next(tmpn))
      {
	
	if (parent[ii] == rimeaddr_node_addr.u8[0]) {
	  a = POWER[7] - tmpn->weight;
	  kk = 7;
	  for (jj=0;jj<7;jj++){
	    b = POWER[jj] - tmpn->weight;
	    kk = (b>0 && a>b)?jj:kk;
	    a = (b>0 && a>b)?b:a;
	  }
	  ll = 3+4*kk > ll?3+4*kk:ll;
	  maxp = a>maxp?a:maxp;
	  maxn++;
	}
	ii++;  
      }
  }
  else {
    if (list_length(neighbor_list) == 1){
      maxn++;
      tmpn = list_head(neighbor_list);
		 
      a = POWER[7] - tmpn->weight;
      kk = 7;
	 
      for (ii=0;ii<7;ii++) {
	b = POWER[ii] - tmpn->weight;
	kk = (b>0 && a>b) ? ii:kk;
	a = (b>0 && a>b) ? b:a;
      }

      ll = 3+4*kk > ll ? 3+4*kk: ll;
      maxp = a > maxp? a:maxp;
    }
    else {
      ll = 31;
    }
  }

 myexit:

  printf("REDELCA %d\n", maxn);


  if (ll < 3){
    ll = 31;
  }


  cc2420_set_txpower(ll);
  //calc the memory peak..
  mem = list_length(neighbor_list)*sizeof(struct neighbor) + list_length(twohopneighs_list)*sizeof(struct twohopneighs) + list_length(edge_list)*(sizeof(struct edge)) + (numofv+1)*sizeof(uint8_t);
   
  for (tmpnn = list_head(twohopneighs_list); tmpnn != NULL; tmpnn = list_item_next(tmpnn)){
    mem = mem + tmpnn->len*(sizeof(float) +sizeof(uint16_t));
  }
  printf("POWER %d\n", ll);
  printf("MEMORY %d\n", mem);
  


  process_post(&b_process, lmst_finish_event, NULL);
 
  //free(tmpe);
  //free(tmpn);
  //free(tmpnn);


  PROCESS_END();

}

//---------------------the PRIM algorithm-----------------------------//
PROCESS_THREAD(prim_process, ev,data)
{

  static uint8_t iv, ie, ik, ss, aa, tmpnode, currentnode, current_ind;
 
  static float mincost, tmpval;
  static uint8_t totalvisited;
 
  static struct neighbor * tmpneigh;
  static struct twohopneighs * tmptwoneigh; 
  static struct edge *tte;


  static uint8_t visited[MAX_NEIGHBORS+1], errorcode;
  static uint8_t s[MAX_NEIGHBORS+1];
  static uint16_t lw[MAX_NEIGHBORS+1];

  PROCESS_BEGIN();
  for (iv=0;iv<MAX_NEIGHBORS+1;iv++){
    visited[iv] = 0;
    parent[iv] = rimeaddr_node_addr.u8[0] & 0xff;
    s[iv] = 0;
    lw[iv] = 0;
  }

	
  PROCESS_YIELD_UNTIL(ev == prim_start_event);


  
  iv=1;   
  for (tmpneigh = list_head(neighbor_list); tmpneigh!=NULL && iv<numofv+1; tmpneigh= list_item_next(tmpneigh)){

    lw[iv++] = (uint16_t)floor(tmpneigh->weight * PRES_CONST / PRES_RANGE);

  }
  

  visited[0] = 1;
  lw[0] = 0;
  s[0] +=1;
  
  totalvisited++;
  
  currentnode = rimeaddr_node_addr.u8[0];
  
  current_ind =0;
  errorcode = 0;

  do {
    mincost = MAXVAL;
    tmpnode = currentnode;
    ss = 0;
    iv=1;

    for (tmpneigh = list_head(neighbor_list); tmpneigh !=NULL && iv<numofv+1; tmpneigh = list_item_next(tmpneigh)){

      tmpval = (float)(lw[iv] * 1000.0 / 200000.00);
	   
      if (visited[iv] == 0) {

	
	if (tmpval <= mincost) {
	  if (s[iv]>=ss){
	    mincost = tmpval;//:mincost;
	    tmpnode = tmpneigh->node_j;//:tmpnode;
	  }
	  ss = s[iv];
	}
      }
      iv++;

    }
    iv=1;
	
    for (tmpneigh = list_head(neighbor_list); tmpneigh!=NULL; tmpneigh = list_item_next(tmpneigh))	{
      if (tmpneigh->node_j == tmpnode) {
	if (visited[iv] ==1) {
	  //you shouldn't be in here!
	  errorcode = 1;
	  break;
	}
	else {
	  visited[iv] = 1;
	  current_ind = iv;
	  currentnode = tmpnode;
	  totalvisited++;
	  //printf("NEIGHBOR: now passing from %d", tmpnode);
	  break;
	}
      }

      iv++;
    }

   
    if (errorcode == 1 || iv == numofv+1){
      printf("NEIGHBOR: ERROR\n");
      break;
    }
    //  printf("NEIGHBOR: currentnode %d:\n", currentnode);
  
    //update the graph's weights, based on the PRIM algorithm
    tte = list_head(edge_list);
    tte = list_item_next(tte);

    for (ik=1;ik<numofv+1; ik++){
	
      if (current_ind == ik) {
	ie=1;
	for (tmptwoneigh = list_head(twohopneighs_list); tmptwoneigh !=NULL && ie<numofv+1; tmptwoneigh = list_item_next(tmptwoneigh)){
	  if (tte->j_ind[ie] == 1){
	    for (aa = 0; aa<tmptwoneigh->len; aa++){
	      if (tmptwoneigh->nn[aa] == currentnode){
		iv=1;
		for (tmpneigh =  list_head(neighbor_list); tmpneigh !=NULL && iv<numofv+1; tmpneigh = list_item_next(tmpneigh)){
		  if 	(tmpneigh->node_j == tmptwoneigh->id && visited[iv] == 0){
		    tmpval = (float)(lw[iv] * 1000.0 / 200000.00);
		    if (tmpval>tmptwoneigh->w[aa]){
		      lw[iv] = (uint16_t)floor(tmptwoneigh->w[aa] * PRES_CONST / PRES_RANGE);
		      parent[iv] = currentnode;
		    }
		    s[iv]++;
		  }
		  iv++;
		}
	      }
	    }

	  }
	  ie++;
	}
	break;
      }
      tte = list_item_next(tte);
    }
  } while (totalvisited<=numofv);
  
  //free(tmpneigh);
  //free(tte);

  //free(tmptwoneigh);
  process_post(&lmst_process, prim_finish_event, NULL);
   
	
  PROCESS_END();
	
}

