/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  USBMIDIBridge.c
 *  Written by Justin Walbeck, 2013
 *
 *  Firmware for a USB to MIDI adapter dongle. Supports bi-directional
 *  communication between a host computer and a hardware controller.
 *  Communication is handled over a fixed 31250 baud serial bus.
 */

#include "USBMIDIBridge.h"

/** LUFA MIDI Class driver interface configuration and state information. This structure is
 *  passed to all MIDI Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface =
	{
		.Config =
			{
				.StreamingInterfaceNumber = 1,
				.DataINEndpoint           =
					{
						.Address          = MIDI_STREAM_IN_EPADDR,
						.Size             = MIDI_STREAM_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint           =
					{
						.Address          = MIDI_STREAM_OUT_EPADDR,
						.Size             = MIDI_STREAM_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** Storage space for a MIDI packet received from the host computer */
MIDI_EventPacket_t MIDIpacket;
/** Storage space for a MIDI packet being constructed to send to the host */
MIDI_EventPacket_t MIDIpacket_out;

/**
 * Initialization routine for the software and hardware side of MIDI comm.
 */
void configure_MIDI() {
  MIDIpacket.Event = 0;
  MIDIpacket.Data1 = 0;
  MIDIpacket.Data2 = 0;
  MIDIpacket.Data3 = 0;
  MIDIpacket_out.Event = 0;
  MIDIpacket_out.Data1 = 0;
  MIDIpacket_out.Data2 = 0;
  MIDIpacket_out.Data3 = 0;
  
  Serial_init();
}

/**
 * The current command being processed. Used by \ref handle_MIDI_in for
 * directing program flow.
 */
uint8_t current_cmd = 0;
/**
 * The number of data bytes received from the controller since the last
 * message was sent to the host.
 */
uint8_t data_ct = 0;
/**
 * The number of data bytes that should be transmitted to the host, based
 * on \ref current_cmd.
 */
uint8_t packet_size = 0;

/** LUT for mapping \ref current_cmd to \ref packet_size. */
const uint8_t MIDI_PACKET_SIZE[15] = {0,0,0,0,0,0,0,0,2,2,2,2,1,1,2};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  SetupHardware();

  sei();
  
  unsigned char c = 0;
  for (;;) {
    //Check if we've received any MIDI messages from the USB line
    if (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &MIDIpacket_out))
    {
      handle_MIDI_out();
    }
    if (Serial_getChar(&c)) {
      handle_MIDI_in(c);
    }
    MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
    USB_USBTask();
  }
}

/**
 * Parses a USB-MIDI packet received from the host computer and passes
 * on the contained MIDI bytes to an attached microcontroller over the
 * hardware USART.
 */
void handle_MIDI_out() {
  //Note we are ignoring the cable number, we only support one cable.
  unsigned char MIDI_CIN = MIDIpacket_out.Event & 0x0F;
  //Based on the Code Index Number only a portion of the incoming data
  //packet may be valid. See the USB Device Class Definition for MIDI Devices.
  switch (MIDI_CIN) {
    case 0x05:
    case 0x0F:
      Serial_sendChar(MIDIpacket_out.Data1);
      break;
    case 0x02:
    case 0x06:
    case 0x0C:
    case 0x0D:
      Serial_sendChar(MIDIpacket_out.Data1);
      Serial_sendChar(MIDIpacket_out.Data2);
      break;
    case 0x03:
    case 0x04:
    case 0x07:
    case 0x08:
    case 0x09:
    case 0x0A:
    case 0x0B:
    case 0x0E:
      Serial_sendChar(MIDIpacket_out.Data1);
      Serial_sendChar(MIDIpacket_out.Data2);
      Serial_sendChar(MIDIpacket_out.Data3);
    case 0x00:
    case 0x01:
    default:
      break;
  }
  return;
}

/**
 * Performs processing on incoming serial MIDI data from an attached
 * microcontroller. Incoming data is parsed for command and data bytes,
 * with a packet being transmitted to the host computer once the proper
 * combination of bytes have been received.
 */
void handle_MIDI_in(uint8_t b) {
  if (b & 0x80) { //Is this a command or data byte
    //New command byte.
    if (b == 0xF0) {
      //SysEx Begin command, handle as special case.
      current_cmd = 0xF0;
      data_ct = 1;
      packet_size = 3;
      MIDIpacket.Event = 0x04;
      MIDIpacket.Data1 = 0xF0;
    } else if (b == 0xF7) {
      //SysEx End message, also a special case
      if (data_ct == 0) {
        MIDIpacket.Event = 0x05;
        MIDIpacket.Data1 = 0xF7;
      } else if (data_ct == 1) {
        MIDIpacket.Event = 0x06;
        MIDIpacket.Data2 = 0xF7;
      } else {
        MIDIpacket.Event = 0x07;
        MIDIpacket.Data3 = 0xF7;
      }
      MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface,&MIDIpacket);
      MIDIpacket.Data1 = 0;
      MIDIpacket.Data2 = 0;
      MIDIpacket.Data3 = 0;
      current_cmd = 0x00;
    } else {
      current_cmd = b;
      data_ct = 0;
      //First, check for a System Common Message.
      if (b >= 0xF6) {
        //Single byte System Common Message, send it
        MIDIpacket.Event = 0x05;
        MIDIpacket.Data1 = b;
        MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface,&MIDIpacket);
        current_cmd = 0;
      } else if (b == 0xF3) {
        //Two byte System Common Message
        MIDIpacket.Event = 0x02;
        MIDIpacket.Data1 = b;
        packet_size = 1;
      } else if (b == 0xF2) {
        //The only three-byte System Common Message
        MIDIpacket.Event = 0x03;
        MIDIpacket.Data1 = b;
        packet_size = 2;
      } else {
        //Channel-specific Message
        current_cmd = b >> 4;
        data_ct = 0;
        packet_size = MIDI_PACKET_SIZE[current_cmd];
        MIDIpacket.Event = current_cmd;
        MIDIpacket.Data1 = b;
      }
    }
  } else {
    //New Data byte
    if (current_cmd == 0x00)
      //Invalid data byte without a corresponding command byte, drop it
      return;
    if (current_cmd == 0xF0) {
      //Currently receiving a SysEx message, special case.
      if (data_ct == 0)
        MIDIpacket.Data1 = b;
      else if (data_ct == 1)
        MIDIpacket.Data2 = b;
      else if (data_ct == 2) {
        MIDIpacket.Data3 = b;
        MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIpacket);
        MIDIpacket.Data1 = 0;
        MIDIpacket.Data2 = 0;
        MIDIpacket.Data3 = 0;
        data_ct = 0;
      }
    } else {
      if (data_ct == 0)
        MIDIpacket.Data2 = b;
      else if (data_ct == 1)
        MIDIpacket.Data3 = b;
      data_ct++;
      //Check if we need to send a packet out.
      if (data_ct == packet_size) {
        //Send the packet along
        MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIpacket);
        //Prep the MIDI object for Running Status
        MIDIpacket.Data2 = 0;
        MIDIpacket.Data3 = 0;
        data_ct = 0;
      }
    }
  }
}


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  //No LED to light up or anything fancy like that
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  //Again, do nothing.
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;
  ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
}

/** Event handler for the library USB Control Request event. */
void EVENT_USB_Device_ControlRequest(void)
{
  MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
}

