/*
 * TODO: add handler for UNICAST_TYPE_NOTIFY_MERGE under unicast
 * TODO: implement sendMergeRequest() which will be called from the above TODO
 * TODO: add handler for sendMergeRequest reception to unicast
 */

/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice,` this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
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
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"

#include <stdio.h>

#include "ghs_simple.h"

/*----------------------------------------------------------------------------------------
 Simplified GHS algorithm:
 --------------
 * Each fragment has a level, starting level is 0.
 * Initially each node is the root of its own fragment.
 * Repeat until all the nodes are in the same fragment (ie, no outgoing edge)
   * Get all neighbours' weights and their fragment IDs (broadcast)
   * The root uses flooding/echo to determine the MWOE of the fragment (u,v).
   * The root sends a message to node u,while forwarding the message all parent-child relations
are inverted, such that u is the new temporary root of the fragment.
   * node u sends merge request to node v.
   * if v also sent a merge request to u, then u/v become root depending on id
   * else node v becomes new parent of node u (and its fragment)
   * newly elected root uses flooding/echo to inform the nodes about the new id

================================================================================
Flooding:
1: The source (root) sends the message to all neighbors.
2: Each other node v upon receiving the message the first time forwards the
message to all (other) neighbors.
3: Upon later receiving the message again (over other edges), a node can discard
the message.
-------------------------------------------------------------------------------
Echo (the simplest convergecast algorithm):
1: A leave sends a message to its parent.
2: If an inner node has received a message from each child, it sends a message
to the parent.
-------------------------------------------------------------------------------
Usually the echo algorithm is paired with the flooding algorithm,
which is used to let the leaves know that they should start the echo
process; this is known as flooding/echo.
================================================================================
   

Original GHS algorithm (not implemented for now)
------------------------------------------
 * MERGE: two fragments of same level L combine to form a new one with level L+1
   - JOIN message sends (L, id). 
   - the joining edge changes status to branch
   - the new root broadcasts (INITIATE, L+1, id)
 * ABSORB: a fragment at level L is absorbed by another fragment at level L' where
   L'>L. The combined fragment has level L'. (each fragment in level L has at least 2^L nodes)
 * To find MWOE from node, each node(i) sends a TEST message through a candidate edge(j) (unicast)
 * Receiver(j) of the TEST message sends back either ACCEPT or REJECT message (unicast):
   1. if (id[i] == id[j]) REJECT (the id here is the fragment id, not the node id)
   2. elif (id[i] != id[j]) AND (level[i] <= level[j]) ACCEPT
   3. elif (id[i] != id[j]) AND (level[i] > level[j]) wait until level[i] == level[j] 
     and then check again to accept/reject
 * Root sends INITIATE message to its own fragment (broadcast) 
 * Nodes send back eligible edges back to root, and root determines MWOE of the fragment (convergecast)
 * Every edge has 3 states: Basic(initial), Branch(tree edge), Rejected(not a tree edge)
     Branch and Rejected are stable attributes (they do not change again)
 */

/* event posted when child responds to mwoe query */ 
process_event_t mwoe_received;

uint8_t state = 0;
uint8_t id; //fragment id
uint8_t numEdges;
int8_t level = 0;
int8_t bestEdge = -1;
int8_t bestWeight = INF_WEIGHT; 
int8_t testEdge = -1;
int8_t hasParent = 0; //initially no parent
rimeaddr_t parent; //address of the parent

uint8_t awaiting_children = 0;
rimeaddr_t mwoe_owner; //our child who is connected to the mwoe, if any
//if NULL, we do not have any children
neighbor reported_mwoe; //this will hold the info about the reported mwoe
neighbor not_found;
static neighbor* min_n = NULL;
uint8_t root;
uint8_t num_children;
uint8_t mwoeIsReceived = 0;
uint8_t mwoeIsRequested = 0;
rimeaddr_t last_added_mwoe_addr;
/*-------------------------------------------------*/
/* FUNCTION DECLARATIONS */
static neighbor* findMinEdge();
static neighbor* addNeighbor(const rimeaddr_t* addr);
static neighbor* addChild(const rimeaddr_t* addr);
//static void clearList(list_t* l); 
static void sendToParent(const neighbor* n, uint8_t type);
static void addChildMWOE();
static void sendMergeRequest(const rimeaddr_t* addr);
static void sendMergeNotify(const rimeaddr_t* addr);
inline uint8_t isRoot(){return root == rimeaddr_node_addr.u8[0];}
inline uint8_t hasChildren(){return num_children > 0;}
static void removeNeighbor(const rimeaddr_t * addr);

/*-------------------------------------------------*/

/* This MEMB() definition defines a memory pool from which we allocate
   neighbor entries. */
MEMB(neighbors_memb, neighbor, MAX_NEIGHBORS);

/* The neighbors_list holds the one-hop neighbors. */
LIST(neighbors_list);
LIST(children_list);

/* These hold the broadcast and unicast structures, respectively. */
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;

/*---------------------------------------------------------------------------*/
PROCESS(broadcast_process, "Broadcast process");

//AUTOSTART_PROCESSES(&broadcast_process, &runicast_process);
AUTOSTART_PROCESSES(&broadcast_process);
/*---------------------------------------------------------------------------*/

/* This function is called whenever a broadcast message is received. */
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct neighbor *n;
  struct broadcast_message *m, msg;

  /* The packetbuf_dataptr() returns a pointer to the first data byte
     in the received packet. */
  m = packetbuf_dataptr();
  switch(m->type){
  case BCAST_TYPE_NEIGHBORS:
    /* Check if we already know this neighbor. */
    n = addNeighbor(from);
    /* If n is NULL, this neighbor was found in our list, but we update its fields anyway*/
    //if(n != NULL){}
    n->weight = packetbuf_attr(PACKETBUF_ATTR_RSSI);
    n->id = m->id;
    n->level = m->level;

    /* Print out a message. */
    /*printf("bcast neighbors message received from %d, RSSI %d, fragment id %d and fragment level %d\n",
	   from->u8[0],
	   packetbuf_attr(PACKETBUF_ATTR_RSSI),
	   n->id,
	   n->level);*/
    break;
  
  case BCAST_TYPE_FIND_MWOE:
    if(hasParent && (rimeaddr_cmp(&parent,from))){ //we only care if we have a parent and the sender is our parent
      printf("mwoe requested from parent...\n");            
      mwoeIsRequested = 1;      
      rimeaddr_copy(&last_added_mwoe_addr, &msg.last_mwoe_addr);
      if(hasChildren()){ //we only bcast if we have children
	awaiting_children = num_children; //we should wait for all our children to reply before we return our own MWOE result
	msg.type = BCAST_TYPE_FIND_MWOE;
	msg.level = m->level; //copy the fragment level to the forwarded message
	msg.id = rimeaddr_node_addr.u8[0]; //copy our id to the forwarded message, so only our children will react
	packetbuf_copyfrom(&msg, sizeof(struct broadcast_message));
	broadcast_send(&broadcast);
      }
      process_post(&broadcast_process, 0, 0); 
    }  
    break;
  }
}
/* This is where we define what function to be called when a broadcast
   is received. We pass a pointer to this structure in the
   broadcast_open() call below. */
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
/*---------------------------------------------------------------------------*/
/* This function is called for every incoming unicast packet. */
static void
recv_uc(struct runicast_conn *c, const rimeaddr_t *from, uint8_t seqno)
{
  struct runicast_message *msg;
  /* Grab the pointer to the incoming data. */ 

  printf("runicast received\n");
  msg = packetbuf_dataptr();

  switch(msg->type){
  case UNICAST_TYPE_MWOE_RESULT:
    if(msg->mwoe_weight > reported_mwoe.weight){ //if we have a new minimum (we should break ties by adding the id value here)
      rimeaddr_copy (&mwoe_owner, from); //change the owner of the mwoe
      reported_mwoe.weight = msg->mwoe_weight; //update the reported mwoe weight to reflect the received value
      rimeaddr_copy (&(reported_mwoe.addr), &(msg->mwoe_addr)); //update the reported mwoe address
      reported_mwoe.id = msg->mwoe_id;    
    }
    mwoeIsReceived = 1;
    printf("We have received MWOE from child %d with ID:%d and RSSI: %d\n",
	   from->u8[0], reported_mwoe.addr.u8[0],reported_mwoe.weight);
    process_post(&broadcast_process, mwoe_received, 0); //we report to the main so that they know we received word from a child
    break;
  case UNICAST_TYPE_NOTIFY_MERGE:
      printf("mwoe notify merge received from ID:%d\n",from->u8[0]);
      if( rimeaddr_cmp(&(msg->mwoe_addr), &(reported_mwoe.addr))  ){ //if mwoe owner is a child
	printf("ID:%d is not one of our children, forwarding to ID:%d\n",msg->mwoe_addr.u8[0],mwoe_owner.u8[0]);
	sendMergeNotify(&(mwoe_owner)); //notify the child
	removeNeighbor(&reported_mwoe.addr); //remove this neighbor if we also have a connection
      }
      else{ //the mwoe reported is one of our own neighbors
	if(min_n != &not_found){
	  sendMergeRequest(&(min_n->addr)); //4. send the merge request to our own mwoe	
	  addChildMWOE(); //move our neighbour mwoe to be our child
	  min_n = &not_found; //clear min_n
	}
      }
    break;
  case UNICAST_TYPE_MERGE_REQUEST: //if we are an unconnected node and receive this message, we should set out parent to be the sender
    printf("Merge requested, ID:%d is our new parent\n",from->u8[0]);
    hasParent = 1; //now we have a parent
    rimeaddr_copy (&parent, from); //change our parent to be the message sender
    removeNeighbor(&parent); //remove parent from neighbors list
    process_post(&broadcast_process, 0, 0);
        //send back ack?
    break;
  }
}

static void
sent_runicast(struct runicast_conn *c, const rimeaddr_t *to, uint8_t retransmissions)
{
  /*  printf("runicast message sent to %d.%d, retransmissions %d\n",
      to->u8[0], to->u8[1], retransmissions);*/
}
static void
timedout_runicast(struct runicast_conn *c, const rimeaddr_t *to, uint8_t retransmissions)
{
  printf("runicast message timed out when sending to %d.%d, retransmissions %d\n",
    to->u8[0], to->u8[1], retransmissions);
}
static const struct runicast_callbacks runicast_callbacks = {recv_uc, sent_runicast, timedout_runicast};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
  static struct etimer et;
  static struct broadcast_message msg;
  static uint8_t count = 0;
  PROCESS_EXITHANDLER(broadcast_close(&broadcast); runicast_close(&runicast);)
    PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);
  runicast_open(&runicast, 146, &runicast_callbacks);

  id = rimeaddr_node_addr.u8[0];
  root = 1; //root is the one with the 0 id, in this simplified centralized version

  not_found.id = -1;
  not_found.weight = -128;

  //rimeaddr_node_addr.u8[0]; //initially each node is th root of its own fragment, so assign own id
  if(isRoot())
    printf("This node is the root!\n");

  msg.type = BCAST_TYPE_NEIGHBORS;
  msg.level = level;
  msg.id = rimeaddr_node_addr.u8[0];
  packetbuf_copyfrom(&msg, sizeof(struct broadcast_message));
  printf("sending bcast_type_neighbors message...\n");

  while(count<3){ //if we do not have any neighbors, keep sending a message every few secs
    count++;
    broadcast_send(&broadcast); 
    etimer_set(&et,CLOCK_SECOND*2 ); //wait for enough time to collect the neighbours
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  //  etimer_set(&et, CLOCK_SECOND *  ); //wait for enough time to collect the neighbours
  //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  if(isRoot())    
    printf("Neighbors found, moving on to find the min edges...\n");
  while(1){ //TODO: for now we do not check if the neighbors are empty, we should stop at some point
    // 1. get all neighbour's weights and their fragment IDs (broadcast)
    
    // 2. The root uses flooding/echo to determine the MWOE of the fragment (u,v).
    //send to all children FIND message, the children should return back their MWOE flood/echo
    etimer_set(&et, CLOCK_SECOND * 4 ); //wait for enough time to collect the neighbours
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));   
    reported_mwoe.weight = INF_WEIGHT; //initialize this to inf so any reported MWOE will be saved
    if ( isRoot() ){
      min_n = findMinEdge(); //find the minimum among our 1-hop neighbors	      
      if ( hasChildren() ){  //if we are the root and if we have children, we need to send the initial message
	printf("sending find mwoe to children...\n");
	msg.type = BCAST_TYPE_FIND_MWOE;
	msg.level = level;
	msg.id = rimeaddr_node_addr.u8[0];
	rimeaddr_copy (&msg.last_mwoe_addr, &last_added_mwoe_addr);
	packetbuf_copyfrom(&msg, sizeof(struct broadcast_message));
	broadcast_send(&broadcast);
	awaiting_children = num_children;
	printf("awaiting result from children...\n");
	while(awaiting_children > 0){ //wait for all children to reply
	  PROCESS_WAIT_EVENT_UNTIL(mwoeIsReceived);
	  awaiting_children--;
	  mwoeIsReceived = 0;
	}
	printf("heard back from children!!\n");
	
	if((reported_mwoe.addr.u8[0] != id) && //if the reported mwoe is not us
	   (reported_mwoe.weight  >  min_n->weight) ) //if the child reported a smaller weight than our own
	  min_n = &reported_mwoe; //use the reported one
	
	if(min_n == &not_found){
	  printf("MST formed!\n");
	  break;
	}
      }
      else{
	printf("root does not have children, so we will use own mwoe...\n");
      }
    }
    else if (!hasParent){ //wait until we have a parent, since we are not the root
      printf("waiting until we have a parent\n");
      PROCESS_WAIT_EVENT_UNTIL(hasParent);
    }
    if(hasParent){
      printf("waiting until mwoe is requested\n");
      PROCESS_WAIT_EVENT_UNTIL(mwoeIsRequested); //wait until the root requests the mwoe
      mwoeIsRequested = 0;
      //remove any previously added mwoe if that exists in our neighbours list
      removeNeighbor(&last_added_mwoe_addr);
      min_n = findMinEdge(); //find the minimum among our 1-hop neighbors
      if( hasChildren() ){
	printf("we are a child and we have children, so we wait for them...\n");
	while(awaiting_children > 0){
	  PROCESS_WAIT_EVENT_UNTIL(mwoeIsReceived);
	  awaiting_children--;
	  mwoeIsReceived = 0;
	}
	if( reported_mwoe.weight  >  min_n->weight ) //if the child reported a smaller weight than our own
	  min_n = &reported_mwoe; //use the reported one
      }
      printf("sending our mwoe to parent with ID:%d and RSSI:%d\n",min_n->addr.u8[0],min_n->weight);
      sendToParent(min_n, UNICAST_TYPE_MWOE_RESULT);
    }
    else if(!isRoot()){
      if(min_n){
	printf("we are a child with no children, send back own MWOE ID:%d with RSSI: %d...\n",
		 min_n->addr.u8[0],min_n->weight);
	  sendToParent(min_n, UNICAST_TYPE_MWOE_RESULT);
      }
      else{
	printf("we do not have any children left, sending to parent null value\n");
	sendToParent(min_n, UNICAST_TYPE_MWOE_RESULT);
      }
    }

    // 3. The root sends a message to node u(the node who found the mwoe),while forwarding the message all parent-child relations
    //    are inverted, such that u is the new temporary root of the fragment. (unicast)
    if ( isRoot() ){
      if ( hasChildren()){
	if( min_n == &reported_mwoe ){  //if mwoe is reported by a child
	  //(maybe we have a connection to mwoe too)
	  sendMergeNotify(&(min_n->addr)); //notify the child (or the new parent)
	  removeNeighbor(&(min_n->addr));
	}
	else{ //if we have children but the mwoe reported is one of our own neighbors
	  if(min_n != &not_found){
	    addChildMWOE(); //add the mwoe as our own child
	    sendMergeRequest(&(min_n->addr)); //4. directly send the merge request to our own connected mwoe
	    min_n = &not_found; //clear min_n
	  }
	}
      }
      else{ //if we do not have any children
	if(min_n != &not_found){
	  addChildMWOE();//add the mwoe as our own child
	  sendMergeRequest(&(min_n->addr)); //4. directly send the merge request to our own connected mwoe
	  min_n = &not_found; //clear min_n
	}
      }
    }
  }
  
  PROCESS_END();
}


static neighbor* findMinEdge() //find the min edge inside fragment (including this node and children if any)
{
  neighbor* n, *min_n = NULL;
  int8_t minWeight = INF_WEIGHT;
  //iterate through all neighbours (and those of our children) and find the minimum weight edge  
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n))
    if(n->weight > minWeight){
      min_n = n;
      minWeight = n->weight;
    }
  /*if(min_n && min_n->addr.u8[0])
    printf("MWOE ID:%d with RSSI: %d\n", min_n->addr.u8[0],minWeight);
  else
  printf("no neighbors left, will ask children for MWOE\n");*/
  if(!min_n)
    min_n = &not_found;
  return min_n;
}

//this function checks the list if the member with addr is inside. 
//if not, adds it to the list and returns a pointer to it
//if it exists, returns NULL
static neighbor* addNeighbor(const rimeaddr_t* addr)
{
  neighbor* n;
  //  printf("adding neighbor with addr %d to list\n", addr->u8[0]);
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
    /* We break out of the loop if the address of the neighbor matches
       the address of the neighbor from which we received this
       broadcast message. */
    if(rimeaddr_cmp(&n->addr, addr)){
      //  printf("neighbor alread in list!\n");
      break;
    }
  }
  /* If n is NULL, this neighbor was not found in our list, and we
     allocate a new struct neighbor from the neighbors_memb memory
     pool. */
  if(n == NULL) {
    //    printf("neighbor not found in list!\n");
    n = memb_alloc(&neighbors_memb);
    //    printf("allocated memory for neighbor, addr: %d!\n",n);
    /* If we could not allocate a new neighbor entry, we give up. We
       could have reused an old neighbor entry, but we do not do this
       for now. */
    if(n != NULL) {
      /* Initialize the fields. */
      rimeaddr_copy(&n->addr, addr);
      /* Place the neighbor on the neighbor list. */
      list_add(neighbors_list, n);
      printf("new neighbor ID:%d added to neighbors list\n",addr->u8[0]);
    }
  }
  return n;
}

static neighbor* addChild(const rimeaddr_t* addr)
{
  neighbor* n;
  // printf("adding neighbor with addr %d as child\n", addr->u8[0]);
  for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
    /* We break out of the loop if the address of the neighbor matches
       the address of the neighbor from which we received this
       broadcast message. */
    if(rimeaddr_cmp(&n->addr, addr)){
      printf("neighbor alread in list!\n");
      break;
    }
  }
  /* If n is NULL, this neighbor was not found in our list, and we
     allocate a new struct neighbor from the neighbors_memb memory
     pool. */
  if(n == NULL) {
    n = memb_alloc(&neighbors_memb);
    /* If we could not allocate a new neighbor entry, we give up. We
       could have reused an old neighbor entry, but we do not do this
       for now. */
    if(n != NULL) {
      /* Initialize the fields. */
      rimeaddr_copy(&n->addr, addr);
      /* Place the neighbor on the neighbor list. */
      list_add(children_list, n);
    }
  }
  return n;
}

//clears the given list and deallocates all members' memory
/*static void clearList(list_t* l)
{
  neighbor* n;
  n = list_pop(*l);
  while(n){
    memb_free(&neighbors_memb, n);
    n = list_pop(*l);
  }
  }*/

//send mwoe result to our parent
static void sendToParent(const neighbor* n, uint8_t type)
{
  struct runicast_message msg;
  //  if( //check if our mwoe is our parent)
  msg.type = type;
  msg.mwoe_addr = n->addr;
  msg.mwoe_id = n->id;
  msg.mwoe_weight = n->weight;
  packetbuf_copyfrom(&msg, sizeof(msg));
  runicast_send(&runicast, &parent, MAX_RETRANSMISSIONS);
}
/*---------------------------------------------------------------------------*/

static void sendMergeNotify(const rimeaddr_t * addr){
  struct runicast_message msg;
  msg.type = UNICAST_TYPE_NOTIFY_MERGE;
  printf("sending merge notify to %d\n",mwoe_owner.u8[0]);
  //  rimeaddr_copy(&msg.mwoe_addr, &n->addr);  
  rimeaddr_copy(&msg.mwoe_addr, addr);  
  // msg.mwoe_id = n->id;
  // msg.mwoe_weight = n->weight;
  packetbuf_copyfrom(&msg, sizeof(msg));
  runicast_send(&runicast, &mwoe_owner, MAX_RETRANSMISSIONS);
}

static void sendMergeRequest(const rimeaddr_t* addr){
  struct runicast_message msg;
  printf("sending merge request to %d\n",addr->u8[0]);
  msg.type = UNICAST_TYPE_MERGE_REQUEST;
  packetbuf_copyfrom(&msg, sizeof(msg));
  runicast_send(&runicast, addr, MAX_RETRANSMISSIONS);
}

static void addChildMWOE(){
  neighbor* n;
  if(min_n != &not_found){
    n = addChild(&min_n->addr);
    //if(n != NULL){}
    n->weight = min_n->weight;
    n->level = min_n->level; //not used
    /* Print out a message. */
    printf("New child connected to the fragment with ID:%d and RSSI: %d, num_children: %d \n",
	   n->addr.u8[0],
	   n->weight,
	   ++num_children);
    list_remove(neighbors_list, min_n);
    rimeaddr_copy(&last_added_mwoe_addr, &n->addr);
    printf("neighbor ID:%d moved from neighbors list to child list\n",n->addr.u8[0]);
  }
}

static void removeNeighbor(const rimeaddr_t * addr){
  neighbor* n;
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
    if(rimeaddr_cmp(&n->addr, addr)){
      printf("neighbor ID:%d removed from neighbors list\n",addr->u8[0]);
      list_remove(neighbors_list, n);
      break;
    }
  }
}
