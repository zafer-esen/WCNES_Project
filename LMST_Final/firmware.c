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
#include <stdbool.h>

#define TIMEFRAME 30
#define RED_WAIT 60
#define MAX_NEIGHBORS 11 //maximum number of neighbors
#define SEND_TIME (random_rand() % (4*CLOCK_SECOND))

/* This structure holds information about 1-hop neighbors. */
struct neighbor {
  struct neighbor *next;
  /*current node, 1-hop neighbour */
  uint16_t node_i;
  uint16_t node_j;
  uint16_t weight;
  uint16_t a;
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
  uint16_t *nn;
  uint16_t *w;
  uint8_t len;
uint8_t weight;
};

float str2float(char *str);

MEMB(neighbor_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbor_list);

MEMB(twohopneighs_memb, struct twohopneighs,MAX_NEIGHBORS);
LIST(twohopneighs_list);

/*---------------------------------------------------------------------------*/
PROCESS(b_process, "Broadcast thread");
AUTOSTART_PROCESSES(&b_process);
/*---------------------------------------------------------------------------*/

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

static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from) {

  char *tmp1;
  char *x=0, *y=0;
  float dx, dy;
  struct neighbor *e = 0;
  struct twohopneighs *ee = 0;
  uint16_t ii, kk, ll, vall, eren, beren;
  uint8_t * tttmp;
  uint8_t rssi_offset = -45;

  
  
  tmp1 = (char *)packetbuf_dataptr();
 
  if(tmp1[0] == 'N'){ //final - neighbourhood rediscovery
    printf("NEIGHBOR %d\n", from->u8[0]);
    return;
  }
 
  if (tmp1[0] == '$') { //1-hop neighbour discovery (initial step)
      tmp1 = strdup((char *)&tmp1[1]);

      x = strtok(tmp1, "#");
      y = strtok((char *)NULL, "#");
 
      dx = str2float(x);
      dy = str2float(y);

      if (list_length(neighbor_list) >= MAX_NEIGHBORS) {
	return;	  
      }
	  	  
      for(e = list_head(neighbor_list); e != NULL; e = list_item_next(e)) {
	if(from->u8[0] == e->node_j) {
	  return;
	}
      }
 
      if (from->u8[0] == rimeaddr_node_addr.u8[0]) {
	return;
      }

      //Place in 1-hop neighbours list	
      e = memb_alloc(&neighbor_memb);
      if(e != NULL) {
	e->node_i = rimeaddr_node_addr.u8[0];
	e->node_j = from->u8[0];
	e->weight = (int16_t)((packetbuf_attr(PACKETBUF_ATTR_RSSI) * -1)+30);
	printf("NEIGHBOR SUCCESSFULLY ADDED TO LIST-> NODE: %d EDGE WEIGHT: %d \n", e->node_j, e->weight);
	list_add(neighbor_list, e);
      }
      return;
    }
  
  //if you have reached at this point, then you are in the 2nd hop neighbourhood discovery.
  if (list_length(twohopneighs_list) >= MAX_NEIGHBORS){
    return;
  }
  
  //place in 2-hop neighbourhood
  ee = memb_alloc(&twohopneighs_memb);
  
  if (ee !=NULL){
    tttmp = (uint8_t *)packetbuf_dataptr();
	
    ll = packetbuf_datalen() / 3; //3 BYTES per neighbour
    ee->id = from->u8[0];
    ee->len = ll;
    ee->nn = malloc(sizeof(uint16_t)*ll);
    ee->w = malloc(sizeof(uint16_t)*ll);
    for (ii=0;ii<ll;ii++){
      ee->nn[ii] =0;
      ee->w[ii] = 0;
    }
	  
    ii = 0;
    for (kk= 0; kk< ll*3;kk=kk+3)
      {
	ee->nn[ii] = tttmp[kk];
	vall = tttmp[kk+1]<<8 | tttmp[kk+2];
	ee->w[ii] = vall; 
	//printf("NEIGHBOR MY TWO HOP ee: ii %d kk: %d \n",ee->w[ii], kk);
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

//Main thread begins
PROCESS_THREAD(b_process, ev, data)
{
  static struct etimer send_timer, redelca_timer;
  static uint32_t start_energy_cpu, start_energy_rx, start_energy_tx;
  static uint32_t end_energy_cpu, end_energy_rx, end_energy_tx;
  static char * msg, * my_x, * my_y, * position;
  static struct neighbor * new;
  static struct neighbor * e;
  static struct neighbor *tmpneigh;
  static uint16_t i, k,val, eren, beren, iv=0;
  static uint8_t *msg1;  
  static struct twohopneighs *twoNeigh;
 
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
  
  position = strdup((char*) data); //strdup: It tries to allocate enough memory to hold the old string (plus a '\0' character to mark the end of the string)
  my_x = strtok(msg, "#");
  my_y = strtok(NULL, "#");
 
  if((position = malloc(strlen("$")+strlen((char *)data)+1)) != NULL){
    position[0] = '\0';   // ensures the memory is an empty string
    strcat(position,"$");
    strcat(position,(char *)data);
  }

  /*Set timers redelca timer to 60s and broadcast timer (1st hop neighbour discovery)*/

  etimer_set(&redelca_timer,RED_WAIT*CLOCK_SECOND);
  etimer_set(&send_timer,((10  + rimeaddr_node_addr.u8[0] % TIMEFRAME) * CLOCK_SECOND));
  
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
  /*Prepare broadcast buffer and broadcast your position*/
 
  packetbuf_clear();
  packetbuf_clear_hdr(); //Clear and reset the header of the packetbuf
  packetbuf_copyfrom(position, strlen(position)); //Copy the position into buffer
  broadcast_send(&broadcast); //Broadcast this position
 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));

  
  //-------------------2-hop neighbour discovery------------------------
  
  if (list_length(neighbor_list) > 0) {
    i=0;

    msg1 = malloc(sizeof(uint8_t)*3*MAX_NEIGHBORS);
    //format is: node id (1B) , weight (2B)

    for(new = list_head(neighbor_list); new != NULL; new = list_item_next(new))
      {
	msg1[i] =  new->node_j & 0xFF;
	i++;
	val = new->weight;
	msg1[i] = val>>8;//msb
	i++;
	msg1[i] = val & 0xFF;//lsb
	i++;
      }

    k = rimeaddr_node_addr.u8[0] % TIMEFRAME;
    k = k * 3;
		
    etimer_set(&redelca_timer, RED_WAIT*2*CLOCK_SECOND); //wait to gather 2 hop neighborhood
    etimer_set(&send_timer, (k+10) * CLOCK_SECOND); //a delay for handling concurrent transmissions.
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    
    packetbuf_clear();
    packetbuf_clear_hdr();
	
    packetbuf_copyfrom(msg1, sizeof(uint8_t)*i);
    broadcast_send(&broadcast);
        
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  
  }
  free(new);
  free(msg1);

  etimer_set(&send_timer, SEND_TIME);
 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
 
  twoNeigh = list_head(twohopneighs_list);

  int size;
  size = list_length(twohopneighs_list) + 1;

  int graph[10][10];
  int arr[10][10];

  for (eren = 0; eren < 10; eren++)
    for (beren = 0; beren < 10; beren++)
      graph[eren][beren] = 0;


  for (eren = 0; eren < 10; eren++)
    for (beren = 0; beren < 10; beren++)
      arr[eren][beren] = 0;

			
  for (twoNeigh = list_head(twohopneighs_list); twoNeigh!=NULL; twoNeigh = list_item_next(twoNeigh)) {
    for(eren=0;eren<(twoNeigh->len);eren++) {
      printf("EDGE FROM NODE %d TO NODE %d -> %d \n",twoNeigh->id, twoNeigh->nn[eren], twoNeigh->w[eren]);
      graph[(twoNeigh->id)-1][(twoNeigh->nn[eren])-1] = twoNeigh->w[eren];
      graph[(twoNeigh->nn[eren])-1][(twoNeigh->id)-1] = twoNeigh->w[eren];
    }
  }

  int a, i, j, indice = 10;
  a = 0;
  /*for(eren=0; eren<10; eren++) {
    for(beren=0; beren<10; beren++) {
      printf("NEIGHBOR ADJ MATRIX graph[%d][%d]=%d \n",eren,beren,graph[eren][beren]);
    }
  }*/


  for(eren=0; eren<10; eren++) {
    if(graph[0][eren]==0) {
      a++;
    }
  }

  if(a == 10) {
    indice = 9;
    for(i=9; i>0; i--) {
      for(j=9; j>0; j--) {
        arr[i-1][j-1] = graph[i][j];
      }
     }
     primMST(indice, arr);
  }
  else {
    primMST(indice, graph);
  }

  /*for(eren=0; eren<10; eren++) {
    for(beren=0; beren<10; beren++) {
      printf("NEIGHBOR CHANGED ADJ MATRIX graph[%d][%d]=%d \n",eren,beren,graph[eren][beren]);
    }
  }*/

  printf("DONE \n");
  PROCESS_END();
}

//******** Local minimum spanning tree calculation with the Prim's algorithm
int minKey(int key[], bool mstSet[], int n)
{
   // Initialize min value
   uint16_t min = INT_MAX, min_index;
   int v;
   for (v = 0; v < n; v++)
     if (mstSet[v] == false && key[v] < min)
         min = key[v], min_index = v;
 
   return min_index;
}
 
// A utility function to print the constructed MST stored in parent[]
void printMST(int parent[], int n, int graph[n][n]) {
   int i;
   printf("LOCAL MINIMUM SPANNING TREE \n");
   printf("EDGE     WEIGHT \n");
   for (i = 1; i < n; i++)
      printf(". %d - %d    %d \n", parent[i]+1, i+1, graph[i][parent[i]]);
}
 
//Construct LMST
void primMST(int y, int graph[y][y])
{
     int parent[y]; // Array to store constructed MST
     int key[y];
     bool mstSet[y];
     int i, count, v;

     // Initialize all keys as MAX (Infinite)
     for (i = 0; i < y; i++)
        key[i] = INT_MAX, mstSet[i] = false;
 
     key[0] = 0;
     parent[0] = -1;
 
     // The MST will have y-1 vertices
     for (count = 0; count < y-1; count++) {
    
        int u = minKey(key, mstSet, y);
 
        mstSet[u] = true;

        for (v = 0; v < y; v++)
          if (graph[u][v] && mstSet[v] == false && graph[u][v] <  key[v])
             parent[v]  = u, key[v] = graph[u][v];
     }
 
     // print the constructed MST
     printMST(parent, y, graph);
}
 
  

