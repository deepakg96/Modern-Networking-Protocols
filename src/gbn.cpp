// Include statements
#include <stdio.h>
#include <string.h>
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
#define TIMEOUT 40

// These keep track of sequence numbers and acknowlegment numbers from both the sides A and B.
struct A_info
{
  int seq_number;
  int ack_number;

  // Needed to determine when to transmit.
  int start_seq_number;
  int next_seq_number;

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

// Packet checksum computing function
int compute_packet_checksum(struct pkt *packet)
{
  // Keep adding the packet data to int variable to generate a random yet reliable checksum
  int i = 0;
  int cksum = 0;
  for (i = 0; i < MSG_SIZE; i++)
  {
    cksum += packet->payload[i];
  }
  // Also add seq_number and ack_number to the checksum
  cksum += packet->seqnum + packet->acknum;
  return cksum;
}

// This is to buffer the messages.
struct msg *message_buffer[MAX_BYTES];

// called from layer 5, passed the data to be sent to other side
void A_output(struct msg message)
{
  // Init a msg struct and copy the data
  struct msg *cur_msg = new msg();
  strncpy(cur_msg->data, message.data, MSG_SIZE);

  // save a copy to our message buffer
  message_buffer[a_data->next_seq_number] = cur_msg;

  if (a_data->next_seq_number < a_data->win_size + a_data->start_seq_number)
  {
    // Build the packet
    struct pkt *outgoing_packet = new pkt();
    strncpy(outgoing_packet->payload, message_buffer[a_data->next_seq_number]->data, MSG_SIZE);
    outgoing_packet->acknum = a_data->ack_number;
    outgoing_packet->seqnum = a_data->next_seq_number;
    outgoing_packet->checksum = compute_packet_checksum(outgoing_packet);

    // Send the packet down
    tolayer3(CONTEXT_A, *outgoing_packet);

    // Start the timer
    if (a_data->next_seq_number == a_data->start_seq_number)
    {
      starttimer(CONTEXT_A, TIMEOUT);
    }
  }
  a_data->next_seq_number += 1;
}

// called from layer 3, when a packet arrives for layer 4
void A_input(struct pkt packet)
{
  // This is when A is getting the packets from B
  // First check the validity of the packet, if it is corrupted, drop the packet.
  if (packet.checksum != compute_packet_checksum(&packet) ||
      packet.acknum < a_data->start_seq_number || 
      packet.acknum > std::min(a_data->start_seq_number + a_data->win_size, a_data->next_seq_number)
      )
  {
    return;
  }

  // Update the sequence numbers
  a_data->start_seq_number = packet.acknum + 1;
  a_data->ack_number = packet.seqnum;

  // Stop the timer
  stoptimer(CONTEXT_A);
}

// called when A's timer goes off
void A_timerinterrupt()
{
  // resend all packets from the window
  for (int pkt_count = a_data->start_seq_number; 
      pkt_count < std::min(a_data->start_seq_number + a_data->win_size, a_data->next_seq_number); 
      pkt_count++)
  {

    // Make the packets
    struct pkt *out_pkt = new pkt();
    out_pkt->seqnum = pkt_count;
    out_pkt->acknum = a_data->ack_number;
    strncpy(out_pkt->payload, message_buffer[pkt_count]->data, MSG_SIZE);
    out_pkt->checksum = compute_packet_checksum(out_pkt);

    // Send out the packets
    tolayer3(CONTEXT_A, *out_pkt);
  }

  // Once the packets are sent, restart the timer.
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
}

// Note that with simplex transfer from a-to-B, there is no B_output()

// called from layer 3, when a packet arrives for layer 4 at B
void B_input(struct pkt packet)
{
  // This is when A is getting the packets from B
  // First check the validity of the packet, if it is corrupted, drop the packet.
  if (packet.checksum != compute_packet_checksum(&packet) || packet.seqnum != b_data->end)
  {
    return;
  }

  // Send the recvd message up
  tolayer5(CONTEXT_B, packet.payload);

  // Update the sequence and ack numbers
  b_data->ack_number = packet.seqnum;
  b_data->seq_number = packet.acknum + 1;

  // Send the ACK
  // Make the ack packet
  struct pkt* out_from_b = new pkt();
  out_from_b->seqnum = b_data->seq_number;
  out_from_b->acknum = b_data->ack_number;
  strncpy(out_from_b->payload, packet.payload, MSG_SIZE);
  out_from_b->checksum = compute_packet_checksum(out_from_b);

  // Send it down
  tolayer3(CONTEXT_B, *out_from_b);

  b_data->end += 1;
}

// the following rouytine will be called once (only) before any other
// entity B routines are called. You can use it to do any initialization
void B_init()
{
  b_data = new B_info();
  b_data->seq_number = 1;
  b_data->ack_number = 1;
  b_data->end = 1;

}