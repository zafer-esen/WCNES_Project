#ifndef GHS_H
#define GHS_H

/*----------------------------------------------*/
/* MESSAGE RELATED DEFINITIONS */
/* This is the structure of broadcast messages. */
struct broadcast_message {
  uint8_t type;
  uint8_t id; //fragment id
  int8_t level; //fragment level
  rimeaddr_t last_mwoe_addr; //the address of the last added mwoe, this is sent so everyone knows the newcomer
};

/* This is the structure of unicast messages. */
struct runicast_message {
  uint8_t type;
  rimeaddr_t mwoe_addr; //node addr
  uint8_t mwoe_id; //fragment id
  int8_t mwoe_weight; //rssi weight
};

/* BCAST message types*/
enum{
  BCAST_TYPE_NEIGHBORS,
  BCAST_TYPE_FIND_MWOE
};

/* UCAST message types */
enum {
  UNICAST_TYPE_MWOE_RESULT,
  UNICAST_TYPE_NOTIFY_MERGE,
  UNICAST_TYPE_MERGE_REQUEST
};
/*-------------------------------------------------*/


/*-------------------------------------------------*/
/* NODE RELATED DEFINITIONS */
/* This structure holds information about neighbors. */
typedef struct neighbor {
  struct neighbor *next;
  /* The ->addr field holds the Rime address of the neighbor. */
  rimeaddr_t addr;
  int8_t weight; //last rssi
  uint8_t state;
  int8_t level; //fragment level
  int8_t id; //fragment id (separate from node id)

}neighbor;

#define MAX_NEIGHBORS 32 /* This #define defines the maximum amount of neighbors we can remember. */
/*-------------------------------------------------*/


/*-------------------------------------------------*/
/* OTHER DEFINITIONS */
#define INF_WEIGHT -128 //int8_t min
#define MAX_RETRANSMISSIONS 255//number of retransmissions for reliable unicast
/*-------------------------------------------------*/

#endif
