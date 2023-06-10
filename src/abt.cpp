// Include some standard libraries
#include <stdio.h>
#include <string.h>
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
#define TIMEOUT 30

// These keep track of sequence numbers and acknowlegment numbers from both the sides A and B.
struct A_info
{
  int seq_number;
  int ack_number;

  // Needed to determine when to transmit.
  int next_seq_number;
  int start_seq_number;
};
struct A_info *a_data;

struct B_info
{
  int seq_number;
  int ack_number;
};
struct B_info *b_data;

// This is to buffer the messages.
struct msg *message_buffer[MAX_BYTES];

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

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
  // Init a new msg structure.
  struct msg *cur_msg = new msg();
  strncpy(cur_msg->data, message.data, MSG_SIZE);

  // Save a copy to the message buffer
  message_buffer[a_data->next_seq_number] = cur_msg;
  
  if (a_data->start_seq_number == a_data->next_seq_number) {
    // Build the packet to send out
    struct pkt* outgoing_packet = new pkt();
    strncpy(outgoing_packet->payload, message_buffer[a_data->next_seq_number]->data, MSG_SIZE);
    outgoing_packet->acknum = a_data->ack_number;
    outgoing_packet->seqnum = a_data->next_seq_number;
    outgoing_packet->checksum = compute_packet_checksum(outgoing_packet);

    // Send the packet down to layer 3
    tolayer3(CONTEXT_A, *outgoing_packet);

    // Set a timeout timer for the sent packet
    starttimer(CONTEXT_A, TIMEOUT);
  }
  // After the packet is sent out, increment the next_seq_number
  a_data->next_seq_number = a_data->next_seq_number + 1;
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  // This is when A is getting the packets from B
  // First check the validity of the packet, if it is corrupted, drop the packet.
  if (packet.checksum != compute_packet_checksum(&packet))
  {
    return;
  }
  
  // Update the packet sequence number and ack number.
  a_data->start_seq_number = packet.acknum + 1;
  a_data->ack_number = packet.seqnum;
  stoptimer(CONTEXT_A);

  // Send ack packet
  if (a_data->start_seq_number < a_data->next_seq_number) {
    // Make the packet
    struct pkt* out_packet = new pkt();
    strncpy(out_packet->payload, message_buffer[a_data->start_seq_number]->data, MSG_SIZE);
    out_packet->acknum = a_data->ack_number;
    out_packet->seqnum = a_data->start_seq_number;
    out_packet->checksum = compute_packet_checksum(out_packet);

    // Send the packet
    tolayer3(CONTEXT_A, *out_packet);
    
    // Start the timer
    starttimer(CONTEXT_A, TIMEOUT);
  }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  // Timer went off. So resend the packet.
  // Make the packet
  struct pkt* outgoing_packet = new pkt();
  strncpy(outgoing_packet->payload, message_buffer[a_data->start_seq_number]->data, MSG_SIZE);
  outgoing_packet->acknum = a_data->ack_number;
  outgoing_packet->seqnum = a_data->start_seq_number;
  outgoing_packet->checksum = compute_packet_checksum(outgoing_packet);

   // Send the packet
  tolayer3(CONTEXT_A, *outgoing_packet);

  // Start the timer again
  starttimer(CONTEXT_A, TIMEOUT);
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
  // Init the new structure and init the members
  a_data = new A_info();
  a_data->seq_number = 1;
  a_data->ack_number = 1;
  a_data->next_seq_number = 1;
  a_data->start_seq_number = 1;
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  // Check validity of the packet, if corrupted, drop it.
  if (packet.checksum != compute_packet_checksum(&packet))
  {
    return;
  }

  // Check if the packet is a duplicate by checking sequence number.
  if(packet.seqnum < b_data->seq_number){
    // Build the ack packet
    struct pkt* old_ack_packet = new pkt();
    old_ack_packet->seqnum = b_data->seq_number;
    old_ack_packet->acknum = b_data->ack_number;
    strncpy(old_ack_packet->payload, packet.payload, MSG_SIZE);
    old_ack_packet->checksum = compute_packet_checksum(old_ack_packet);

    // Send the ack packet
    tolayer3(CONTEXT_B, *old_ack_packet);

    return;
  }
  else
  {

    // Send the message upstairs
    tolayer5(1, packet.payload);

    // Now send the acknowledgement out.
    // Update sequence number and acknowledgement number.
    b_data->ack_number = packet.seqnum;
    b_data->seq_number = packet.acknum + 1;

    // Build the packet
    struct pkt* ack_packet = new pkt();
    ack_packet->seqnum = b_data->seq_number;
    ack_packet->acknum = b_data->ack_number;
    strncpy(ack_packet->payload, packet.payload, MSG_SIZE);
    ack_packet->checksum = compute_packet_checksum(ack_packet);

    // Send the packet out
    tolayer3(CONTEXT_B, *ack_packet);  
  }

}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
  // Init the new strcuture and init the members.
  b_data = new B_info();
  b_data->seq_number = 1;
  b_data->ack_number = 1;
}
