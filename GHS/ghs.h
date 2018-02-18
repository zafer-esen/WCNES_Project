#ifndef GHS_H
#define GHS_H

/*----------------------------------------------*/
/* MESSAGE RELATED DEFINITIONS */
/* This is the structure of broadcast messages. */
struct broadcast_message {
  uint8_t bcast_type;
  uint8_t id; //fragment id
  int8_t level; //fragment level
};

/* This is the structure of unicast messages. */
struct unicast_message {
  uint8_t ucast_type;
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
  UNICAST_TYPE_MWOE_RESULT
};
/*-------------------------------------------------*/

/*-------------------------------------------------*/
/* FUNCTION DECLARATIONS */
inline uint8_t isRoot(){return root == rimeaddr_node_addr.u8[0];}
inline uint8_t hasChildren(){return num_children > 0;}
static neighbor* findMinEdge();
static neighbor* addToList(list_t* l, const rime_addr_t* addr);
static void clearList(list_t* l); 
static void sendToParent(const node* n);
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
#define INF_WEIGHT 127 //int8_t max, rssi values are in the negative, so this can be seen as inf
/*-------------------------------------------------*/

#endif
