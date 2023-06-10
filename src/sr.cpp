// Include statements
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <algorithm>
#include "../include/simulator.h"

/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/

// #defines to avoid free floating numbers.
#define MSG_SIZE 20
#define CONTEXT_A 0
#define CONTEXT_B 1
#define MAX_BYTES 9999
#define TIMEOUT 20
#define SR_INTERVAL 50


// These keep track of sequence numbers and acknowlegment numbers from both the sides A and B.
struct A_info
{
  int seq_number;
  int ack_number;

  // Needed to determine when to transmit.
  int next_seq_number;
  int start_seq_number;

  // Window size
  int win_size;
};
struct A_info *a_data;

struct B_info
{
  int seq_number;
  int ack_number;
  int end;
};
struct B_info *b_data;


// Message buffer helper struct
struct saved_msg {
   int is_acked;
   int elapsed_time;
   struct msg* buf;
};
struct saved_msg* saved_buffer[MAX_BYTES];
struct msg* msg_buffer[MAX_BYTES];

// Packet checksum computing function
int compute_packet_checksum(struct pkt *packet)
{
  // Keep adding the packet data to int variable to generate a random yet reliable checksum
  int i = 0;
  int cksum = 0;
  for(i=0; i<MSG_SIZE; i++)
  {
    cksum += packet->payload[i];
  }
  // Also add seq_number and ack_number to the checksum
  cksum += packet->seqnum + packet->acknum;
  return cksum;
}

// called from layer 5, passed the data to be sent to other side 
void A_output(struct msg message)
{
  // Init the new msg structure
  struct msg* curr_msg = new msg();
  strncpy(curr_msg->data, message.data, MSG_SIZE);
  struct saved_msg* mc = new saved_msg();
  mc->buf = curr_msg;
  mc->is_acked = 0;
  mc->elapsed_time = get_sim_time();
  saved_buffer[a_data->next_seq_number] = mc;
  
  if (a_data->next_seq_number < a_data->start_seq_number + a_data->win_size)
  {
    // Build the packet
    struct pkt *outgoing_packet = new pkt();
    strncpy(outgoing_packet->payload, saved_buffer[a_data->next_seq_number]->buf->data, MSG_SIZE);
    outgoing_packet->acknum = a_data->ack_number;
    outgoing_packet->seqnum = a_data->next_seq_number;
    outgoing_packet->checksum = compute_packet_checksum(outgoing_packet);

    // Send the packet down
    tolayer3(CONTEXT_A, *outgoing_packet);

  }
  a_data->next_seq_number += 1;
}

// called from layer 3, when a packet arrives for layer 4 
void A_input(struct pkt packet)
{
  // Check for packet's validity
  if (packet.checksum != compute_packet_checksum(&packet) ||
      packet.acknum < a_data->start_seq_number || 
      packet.acknum > std::min(a_data->start_seq_number + a_data->win_size, a_data->next_seq_number))
  {
    return;
  }

  // Update the is_acked field.
  saved_buffer[packet.acknum]->is_acked = 1;
  
  // This updates both ack and seq numbers
  if(a_data->start_seq_number == packet.acknum)
  {
    a_data->start_seq_number = packet.acknum + 1;
  }
  a_data->ack_number = packet.seqnum;

}

// called when A's timer goes off 
void A_timerinterrupt()
{
  int current_time = get_sim_time();
  for (int pkts_c = a_data->start_seq_number; 
      pkts_c < std::min(a_data->start_seq_number + a_data->win_size, a_data->next_seq_number); 
      pkts_c++)
  {
    if(saved_buffer[pkts_c]->elapsed_time + SR_INTERVAL < current_time && saved_buffer[pkts_c]->is_acked == 0)
    {
      // Make packets
      struct pkt *send_pkt = new pkt();
      strncpy(send_pkt->payload, saved_buffer[pkts_c]->buf->data, MSG_SIZE);
      send_pkt->seqnum = pkts_c;
      send_pkt->acknum = a_data->ack_number;
      send_pkt->checksum = compute_packet_checksum(send_pkt);

      // Send it down
      tolayer3(CONTEXT_A, *send_pkt);

      // Upadte the time
      saved_buffer[pkts_c]->elapsed_time = current_time;

    }
  }
  // Start the timer.
  starttimer(CONTEXT_A, TIMEOUT);
}  

// the following routine will be called once (only) before any other 
// entity A routines are called. You can use it to do any initialization 
void A_init()
{
  a_data = new A_info();
  a_data->seq_number = 1;
  a_data->ack_number = 1;
  a_data->start_seq_number = 1;
  a_data->next_seq_number = 1;

  // Update the window size passed by the cmd line arguments
  a_data->win_size = getwinsize();

  starttimer(CONTEXT_A, TIMEOUT);
}

// Note that with simplex transfer from a-to-B, there is no B_output() 

// called from layer 3, when a packet arrives for layer 4 at B
void B_input(struct pkt packet)
{

  // Save the msg info into a temp buffer
  char temp_buffer[20];

  // Check for validitiy of the packet
  if (packet.checksum != compute_packet_checksum(&packet) ||
      packet.seqnum > b_data->end + a_data->win_size ||
      packet.seqnum < b_data->end - a_data->win_size)
  {
    return;
  }

  // Save a copy to msg_buffer
  struct msg* to_msg_array = new msg();
  strncpy(to_msg_array->data, packet.payload, MSG_SIZE);
  msg_buffer[packet.seqnum] = to_msg_array;

  if(b_data->end == packet.seqnum){
    for (int buffers = b_data->end ; msg_buffer[b_data->end ] != NULL ; b_data->end++)
    {
      // Copy to temp_buffer and pass it up.
      strncpy(temp_buffer, msg_buffer[b_data->end]->data, MSG_SIZE);
      tolayer5(CONTEXT_B, temp_buffer);
    }
  }

  // Send ack back.
  // Build the packet.
  struct pkt *ack_pkt = new pkt();
  strncpy(ack_pkt->payload, temp_buffer, MSG_SIZE);
  ack_pkt->seqnum = packet.acknum + 1;
  ack_pkt->acknum = packet.seqnum;
  ack_pkt->checksum = compute_packet_checksum(ack_pkt);

  // Send it down.
  tolayer3(CONTEXT_B, *ack_pkt);
}

// the following rouytine will be called once (only) before any other 
// entity B routines are called. You can use it to do any initialization 
void B_init()
{
  // Init members of B_info
  b_data = new B_info();
  b_data->seq_number = 1;
  b_data->ack_number = 1;
  b_data->end = 1;

  // Zero init
  bzero(msg_buffer, sizeof(msg_buffer));
}

