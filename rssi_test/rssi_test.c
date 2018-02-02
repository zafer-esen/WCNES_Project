#include <stdio.h>
#include "contiki.h"
#include "serial-shell.h"
#include "shell-ps.h"
#include "shell-file.h"
#include "shell-text.h"
#include "net/rime.h"
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
#include "random.c"
#include "collect.h"

static struct collect_conn tc;

PROCESS(example_collect_process, "RSS Measurement");
AUTOSTART_PROCESSES(&example_collect_process);

static void recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  static signed char rss;
  static signed char rss_val;
  static signed char rss_offset;
  printf("Sink got message from %d.%d, seqno %d, hops %d: len %d '%s'\n",originator->u8[0], originator->u8[1],seqno, hops,packetbuf_datalen(),
         (char *)packetbuf_dataptr());
  rss_val = cc2420_last_rssi;
  rss_offset=-45;
  rss=rss_val + rss_offset;
  printf("RSSI of Last Packet Received is %d\n",rss);
}

static const struct collect_callbacks callbacks = { recv };

PROCESS_THREAD(example_collect_process, ev, data)
{
  static struct etimer periodic;
  static struct etimer et;
  
  PROCESS_BEGIN();

  collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

  if(rimeaddr_node_addr.u8[0] == 1 && rimeaddr_node_addr.u8[1] == 0) 
  {
    printf("I am sink\n");
    collect_set_sink(&tc, 1);
  }
  else
    printf("I am not sink\n");

  /* Allow some time for the network to settle. */
  etimer_set(&et, 120 * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));

  while(1) 
  {

    /* Send a packet every 1 seconds. */
    if(etimer_expired(&periodic)) 
    {
      etimer_set(&periodic, CLOCK_SECOND * 1 );
      etimer_set(&et, random_rand() % (CLOCK_SECOND * 1));
    }

    PROCESS_WAIT_EVENT();


    if(etimer_expired(&et)) 
    {
      static rimeaddr_t oldparent;
      const rimeaddr_t *parent;
      if(rimeaddr_node_addr.u8[0] != 1 )
      {
        printf("Sending\n");
        packetbuf_clear();
        packetbuf_set_datalen(sprintf(packetbuf_dataptr(),"%s", "Fight On") + 1);
        collect_send(&tc, 15);

        parent = collect_parent(&tc);
        if(!rimeaddr_cmp(parent, &oldparent)) 
        {
           if(!rimeaddr_cmp(&oldparent, &rimeaddr_null))
           {
              printf("#L %d 0\n", oldparent.u8[0]);
           }
           if(!rimeaddr_cmp(parent, &rimeaddr_null)) 
           {
              printf("#L %d 1\n", parent->u8[0]);
           }
           
           rimeaddr_copy(&oldparent, parent);
        }
      }
    }

  } //end of while

  PROCESS_END();
} //end of process thread
