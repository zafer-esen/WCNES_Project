#include "contiki.h"
#include <stdio.h>
#include <limits.h>
#include <stdbool.h>
 
// Number of vertices in the graph
#define V 8

PROCESS(prim_process, "Prim's algorithm");
AUTOSTART_PROCESSES(&prim_process);

PROCESS_THREAD(prim_process, ev, data) {

PROCESS_BEGIN();

	//Adjacency matrix of the undirected graph
   int graph[V][V] = {{0, 6, 12, 0, 0, 0, 0, 0},
                      {6, 0, 5, 0, 14, 0, 0, 8},
                      {12, 5, 0, 9, 0, 7, 0, 0},
                      {0, 0, 9, 0, 0, 0, 0, 0},
                      {0, 14, 0, 0, 0, 0, 0, 3},
		      {0, 0, 7, 0, 0, 0, 15, 10},
		      {0, 0, 0, 0, 0, 15, 0, 0},
		      {0, 8, 0, 0, 3, 10, 0, 0}
                     };
 
    // Print the solution
    primMST(graph);
 

PROCESS_END();


}




 
// A utility function to find the vertex with minimum key value, from
// the set of vertices not yet included in MST
int minKey(int key[], bool mstSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
	int v;
   for (v = 0; v < V; v++)
     if (mstSet[v] == false && key[v] < min)
         min = key[v], min_index = v;
 
   return min_index;
}
 
// A utility function to print the constructed MST stored in parent[]
void printMST(int parent[], int n, int graph[V][V])
{
   int i;
   printf("Edge   Weight\n");
   for (i = 1; i < V; i++)
      printf("%d - %d    %d \n", parent[i], i, graph[i][parent[i]]);
}
 
// Function to construct and print MST for a graph represented using adjacency
// matrix representation
void primMST(int graph[V][V])
{
     int parent[V]; // Array to store constructed MST
     int key[V];   // Key values used to pick minimum weight edge in cut
     bool mstSet[V];  // To represent set of vertices not yet included in MST
     int i, count, v;
     // Initialize all keys as INFINITE
     for (i = 0; i < V; i++)
        key[i] = INT_MAX, mstSet[i] = false;
 
     // Always include first 1st vertex in MST.
     key[0] = 0;     // Make key 0 so that this vertex is picked as first vertex
     parent[0] = -1; // First node is always root of MST 
 
     // The MST will have V vertices
     for (count = 0; count < V-1; count++)
     {
        // Pick the minimum key vertex from the set of vertices
        // not yet included in MST
        int u = minKey(key, mstSet);
 
        // Add the picked vertex to the MST Set
        mstSet[u] = true;
 
        // Update key value and parent index of the adjacent vertices of
        // the picked vertex. Consider only those vertices which are not yet
        // included in MST
        for (v = 0; v < V; v++)
           // graph[u][v] is non zero only for adjacent vertices of m
           // mstSet[v] is false for vertices not yet included in MST
           // Update the key only if graph[u][v] is smaller than key[v]
          if (graph[u][v] && mstSet[v] == false && graph[u][v] <  key[v])
             parent[v]  = u, key[v] = graph[u][v];
     }
 
     // print the constructed MST
     printMST(parent, V, graph);
}
 