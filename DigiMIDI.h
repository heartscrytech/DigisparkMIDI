/*
 * Based on Obdev's AVRUSB code and under the same license.
 *
 * HeartCry Music Midi Driver:
 * -    Provides a simple midi-send port and standard HID-MIDI driver for ATtiny85 and arduino. Tested
 *      with knock-off digisparks.
 *      Based on V-USB-MIDI-0.2
 *
 *      Tested on   Mac OS X 10.10
 *                  iOS 9.0.0
 *
 * TODO: Make a proper file header. :-)
 * Modified for Digispark by Digistump
 */
#ifndef __DigiMIDI_h__
#define __DigiMIDI_h__

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <string.h>

#include "usbdrv.h"
#include "scancode-ascii-table.h"

// TODO: Work around Arduino 12 issues better.
//#include <WConstants.h>
//#undef int()

typedef uint8_t byte;


#define BUFFER_SIZE 2 // Minimum of 2: 1 for modifiers + 1 for keystroke 


static uchar    idleRate;           // in 4 ms units 

//-------------------------------------------------------------
//-------------------------- MIDI DEFS ------------------------
#define MIDI_CHANNEL_OMNI       0
#define MIDI_CHANNEL_OFF        17 // and over

#define MIDI_PITCHBEND_MIN      -8192
#define MIDI_PITCHBEND_MAX      8191
typedef byte StatusByte;
typedef byte DataByte;
typedef byte Channel;
typedef byte FilterMode;

// Enumeration of MIDI types
enum MidiType
{
    InvalidType           = 0x00,    ///< For notifying errors
    NoteOff               = 0x80,    ///< Note Off
    NoteOn                = 0x90,    ///< Note On
    AfterTouchPoly        = 0xA0,    ///< Polyphonic AfterTouch
    ControlChange         = 0xB0,    ///< Control Change / Channel Mode
    ProgramChange         = 0xC0,    ///< Program Change
    AfterTouchChannel     = 0xD0,    ///< Channel (monophonic) AfterTouch
    PitchBend             = 0xE0,    ///< Pitch Bend
    SystemExclusive       = 0xF0,    ///< System Exclusive
    TimeCodeQuarterFrame  = 0xF1,    ///< System Common - MIDI Time Code Quarter Frame
    SongPosition          = 0xF2,    ///< System Common - Song Position Pointer
    SongSelect            = 0xF3,    ///< System Common - Song Select
    TuneRequest           = 0xF6,    ///< System Common - Tune Request
    Clock                 = 0xF8,    ///< System Real Time - Timing Clock
    Start                 = 0xFA,    ///< System Real Time - Start
    Continue              = 0xFB,    ///< System Real Time - Continue
    Stop                  = 0xFC,    ///< System Real Time - Stop
    ActiveSensing         = 0xFE,    ///< System Real Time - Active Sensing
    SystemReset           = 0xFF,    ///< System Real Time - System Reset
};
#define InvalidType         0x00
#define NoteOff             0x80
#define NoteOn              0x90
#define ControlChange       0xB0
#define ProgramChange       0xC0
#define AfterTouchPoly      0xA0
#define AfterTouchChannel   0xD0
#define PitchBend           0xE0

//-------------------------------------------------------------
//-------------------------------------------------------------

// This descriptor is based on http://www.usb.org/developers/devclass_docs/midi10.pdf
//
// Appendix B. Example: Simple MIDI Adapter (Informative)
// B.1 Device Descriptor
//
const PROGMEM char deviceDescrMIDI[19] = {	/* USB device descriptor */
    18,			/* sizeof(usbDescriptorDevice): length of descriptor in bytes */
    USBDESCR_DEVICE,	/* descriptor type */
    0x10, 0x01,		/* USB version supported */
    0,			/* device class: defined at interface level */
    0,			/* subclass */
    0,			/* protocol */
    8,			/* max packet size */
    USB_CFG_VENDOR_ID,	/* 2 bytes */
    USB_CFG_DEVICE_ID,	/* 2 bytes */
    USB_CFG_DEVICE_VERSION,	/* 2 bytes */
    1,			/* manufacturer string index */
    2,			/* product string index */
    0,			/* serial number string index */
    1,			/* number of configurations */
};

// B.2 Configuration Descriptor
const PROGMEM char configDescrMIDI[] = {	/* USB configuration descriptor */
    9,			/* sizeof(usbDescrConfig): length of descriptor in bytes */
    USBDESCR_CONFIG,	/* descriptor type */
    101, 0,			/* total length of data returned (including inlined descriptors) */
    2,			/* number of interfaces in this configuration */
    1,			/* index of this configuration */
    0,			/* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
    USBATTR_SELFPOWER,	/* attributes */
#else
    USBATTR_BUSPOWER,	/* attributes */
#endif
    USB_CFG_MAX_BUS_POWER / 2,	/* max USB current in 2mA units */
    
    // B.3 AudioControl Interface Descriptors
    // The AudioControl interface describes the device structure (audio function topology)
    // and is used to manipulate the Audio Controls. This device has no audio function
    // incorporated. However, the AudioControl interface is mandatory and therefore both
    // the standard AC interface descriptor and the classspecific AC interface descriptor
    // must be present. The class-specific AC interface descriptor only contains the header
    // descriptor.
    
    // B.3.1 Standard AC Interface Descriptor
    // The AudioControl interface has no dedicated endpoints associated with it. It uses the
    // default pipe (endpoint 0) for all communication purposes. Class-specific AudioControl
    // Requests are sent using the default pipe. There is no Status Interrupt endpoint provided.
    /* AC interface descriptor follows inline: */
    9,			/* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE,	/* descriptor type */
    0,			/* index of this interface */
    0,			/* alternate setting for this interface */
    0,			/* endpoints excl 0: number of endpoint descriptors to follow */
    1,			/* */
    1,			/* */
    0,			/* */
    0,			/* string index for interface */
    
    // B.3.2 Class-specific AC Interface Descriptor
    // The Class-specific AC interface descriptor is always headed by a Header descriptor
    // that contains general information about the AudioControl interface. It contains all
    // the pointers needed to describe the Audio Interface Collection, associated with the
    // described audio function. Only the Header descriptor is present in this device
    // because it does not contain any audio functionality as such.
    /* AC Class-Specific descriptor */
    9,			/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
    36,			/* descriptor type */
    1,			/* header functional descriptor */
    0x0, 0x01,		/* bcdADC */
    9, 0,			/* wTotalLength */
    1,			/* */
    1,			/* */
    
    // B.4 MIDIStreaming Interface Descriptors
    
    // B.4.1 Standard MS Interface Descriptor
    /* interface descriptor follows inline: */
    9,			/* length of descriptor in bytes */
    USBDESCR_INTERFACE,	/* descriptor type */
    1,			/* index of this interface */
    0,			/* alternate setting for this interface */
    2,			/* endpoints excl 0: number of endpoint descriptors to follow */
    1,			/* AUDIO */
    3,			/* MS */
    0,			/* unused */
    0,			/* string index for interface */
    
    // B.4.2 Class-specific MS Interface Descriptor
    /* MS Class-Specific descriptor */
    7,			/* length of descriptor in bytes */
    36,			/* descriptor type */
    1,			/* header functional descriptor */
    0x0, 0x01,		/* bcdADC */
    65, 0,			/* wTotalLength */
    
    // B.4.3 MIDI IN Jack Descriptor
    6,			/* bLength */
    36,			/* descriptor type */
    2,			/* MIDI_IN_JACK desc subtype */
    1,			/* EMBEDDED bJackType */
    1,			/* bJackID */
    0,			/* iJack */
    
    6,			/* bLength */
    36,			/* descriptor type */
    2,			/* MIDI_IN_JACK desc subtype */
    2,			/* EXTERNAL bJackType */
    2,			/* bJackID */
    0,			/* iJack */
    
    //B.4.4 MIDI OUT Jack Descriptor
    9,			/* length of descriptor in bytes */
    36,			/* descriptor type */
    3,			/* MIDI_OUT_JACK descriptor */
    1,			/* EMBEDDED bJackType */
    3,			/* bJackID */
    1,			/* No of input pins */
    2,			/* BaSourceID */
    1,			/* BaSourcePin */
    0,			/* iJack */
    
    9,			/* bLength of descriptor in bytes */
    36,			/* bDescriptorType */
    3,			/* MIDI_OUT_JACK bDescriptorSubtype */
    2,			/* EXTERNAL bJackType */
    4,			/* bJackID */
    1,			/* bNrInputPins */
    1,			/* baSourceID (0) */
    1,			/* baSourcePin (0) */
    0,			/* iJack */
    
    
    // B.5 Bulk OUT Endpoint Descriptors
    
    //B.5.1 Standard Bulk OUT Endpoint Descriptor
    9,			/* bLenght */
    USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
    0x1,			/* bEndpointAddress OUT endpoint number 1 */
    3,			/* bmAttributes: 2:Bulk, 3:Interrupt endpoint */
    8, 0,			/* wMaxPacketSize */
    10,			/* bIntervall in ms */
    0,			/* bRefresh */
    0,			/* bSyncAddress */
    
    // B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
    5,			/* bLength of descriptor in bytes */
    37,			/* bDescriptorType */
    1,			/* bDescriptorSubtype */
    1,			/* bNumEmbMIDIJack  */
    1,			/* baAssocJackID (0) */
    
    
    //B.6 Bulk IN Endpoint Descriptors
    
    //B.6.1 Standard Bulk IN Endpoint Descriptor
    9,			/* bLenght */
    USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
    0x81,			/* bEndpointAddress IN endpoint number 1 */
    3,			/* bmAttributes: 2: Bulk, 3: Interrupt endpoint */
    8, 0,			/* wMaxPacketSize */
    10,			/* bIntervall in ms */
    0,			/* bRefresh */
    0,			/* bSyncAddress */
    
    // B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
    5,			/* bLength of descriptor in bytes */
    37,			/* bDescriptorType */
    1,			/* bDescriptorSubtype */
    1,			/* bNumEmbMIDIJack (0) */
    3,			/* baAssocJackID (0) */
};


class DigiMIDIDevice {
 public:
    DigiMIDIDevice () {
        cli();
        usbDeviceDisconnect();
        _delay_ms(500); //250
        usbDeviceConnect();

        //wdt_enable(WDTO_1S);
        usbInit();
          
        sei();
    }

    // [KEYBOARD] Not sure if we need this
    // This should be called in a sketch's main loop. We need this to poll our
    // interface
    void update() {
        //wdt_reset(); //wdt_enable()
        usbPoll();
        
        if (usbInterruptIsReady()) {
            //TODO: Read for midi notes and then call a callback based on the data
            //we do our own read here and send to callback if the user
            //has defined a read callback function.
        
        }
    }
	
    // [KEYBOARD] delay while updating until we are finished delaying
    void delay(long milli) {
        unsigned long last = millis();
      while (milli > 0) {
        unsigned long now = millis();
        milli -= now - last;
        last = now;
        update();
      }
    }
   
private:
    /*! \brief Send a Real Time (one byte) message.
     
     \param inType    The available Real Time types are:
     Start, Stop, Continue, Clock, ActiveSensing and SystemReset.
     You can also send a Tune Request with this method.
     @see MidiType
     */
    void sendRealTime(byte /*MidiType*/ inType, bool wait=false) {
        if(wait) {
            while(!usbInterruptIsReady()) {
                usbPoll();
                delay(5);
            }
        }
        
        switch (inType)
        {
            case TuneRequest: // Not really real-time, but one byte anyway.
            case Clock:
            case Start:
            case Stop:
            case Continue:
            case ActiveSensing:
            case SystemReset:
                
                uchar midiMsg[2];
                midiMsg[0]=0x0f;    //Note sure why I need this...
                midiMsg[1]=inType;

                usbSetInterrupt(midiMsg, sizeof(midiMsg));
                break;
            
            default:
                // Invalid Real Time marker
                break;
        }
    }

public:
    //See arduino MIDI for details. However this is not a complete drop-in but works in some ways simular
    void send(byte /*MidiType*/ inType,
              byte /*DataByte*/ inData1=0x00,
              byte /*DataByte*/ inData2=0x00,
              byte /*Channel*/  inChannel=0x01) {
        while(!usbInterruptIsReady()) {
            usbPoll();
            delay(5);
        }
        
        // Then test if channel is valid
        if (inChannel >= MIDI_CHANNEL_OFF  ||
            inChannel == MIDI_CHANNEL_OMNI ||
            inType < NoteOff)
        {
            return; // Don't send anything
        }
        
        if (inType <= 0xE0/*PitchBend*/)  // Channel messages
        {
            // Protection: remove MSBs on data
            inData1 &= 0x7f;
            inData2 &= 0x7f;
            
            const byte status = getStatus(inType, inChannel);
            
            uchar midiMsg[4];
            uchar iii;
            iii = 0;
            midiMsg[iii++] = 0x08; //0x08   //what is this?
            midiMsg[iii++] = status;        //Message type
            midiMsg[iii++] = inData1;       //Data1? //Key
            midiMsg[iii++] = inData2;
            
            //first send the status byte.
            usbSetInterrupt(midiMsg, iii);
            
            // Then send data
            /*mSerial.write(inData1);
            if (inType != ProgramChange && inType != AfterTouchChannel)
                mSerial.write(inData2);
            */
            return;
        }
        else if (inType >= TuneRequest && inType <= SystemReset) {
            sendRealTime(inType); // System Real-time and 1 byte.
            
        }
    }
    
    //For internal testing
    void sendNoteOn(int key, int velocity=0, uchar channel=1) {
        send(NoteOn,key,velocity,channel);
    }
    void sendNoteOff(int key, int velocity=0, uchar channel=1) {
        send(NoteOff,key,velocity,channel);
    }
    void sendProgramChange(int inProgramNumber, uchar channel=1) {
        send(ProgramChange,inProgramNumber,0,channel);
    }
    void sendControlChange(int inControlNumber, int inControlValue=0, uchar channel=1) {
        send(ControlChange,inControlNumber,inControlValue,channel);
    }
    void sendPolyPressure(int inNoteNumber, int inPressure, uchar channel=1) {
        send(AfterTouchPoly,inNoteNumber,inPressure,channel);
    }
    void sendAfterTouch(int inPressure, uchar channel=1) {
        send(AfterTouchChannel, inPressure, 0, channel);
    }
    void sendPitchBend(int inPitchValue, uchar channel=1) {
        const unsigned bend = inPitchValue - 0;/*MIDI_PITCHBEND_MIN;*/
        send(PitchBend, (bend & 0x7f), (bend >> 7) & 0x7f, channel);
    }
    void sendPitchBend(double inPitchValue, uchar channel=1) {
        const int value = inPitchValue;// * MIDI_PITCHBEND_MAX;
        sendPitchBend(value, channel);
    }
    
    /*StatusByte*/
    byte getStatus(byte /*MidiType*/ inType,
                                 byte /*Channel*/ inChannel) const {
        return ((byte)inType | ((inChannel - 1) & 0x0f));
    }
    
};

DigiMIDIDevice DigiMIDI = DigiMIDIDevice();

#ifdef __cplusplus
extern "C" {
#endif 
  
    // ------ MIDI --------
    // Now for the midi fun
    // --------------------
    
    static uchar sendEmptyFrame;
    
    uchar usbFunctionDescriptor(usbRequest_t * rq)
    {
        
        if (rq->wValue.bytes[1] == USBDESCR_DEVICE) {
            usbMsgPtr = (uchar *) deviceDescrMIDI;
            return sizeof(deviceDescrMIDI);
        } else {		/* must be config descriptor */
            usbMsgPtr = (uchar *) configDescrMIDI;
            return sizeof(configDescrMIDI);
        }
    }
    
    
    /* ------------------------------------------------------------------------- */
    /* ----------------------------- USB interface ----------------------------- */
    /* ------------------------------------------------------------------------- */
    
    uchar usbFunctionSetup(uchar data[8])
    {
        usbRequest_t    *rq = (usbRequest_t *)((void *)data);
        
        // DEBUG LED
        //PORTC ^= 0x01;
        
        if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
            
            // Prepare bulk-in endpoint to respond to early termination
            if ((rq->bmRequestType & USBRQ_DIR_MASK) ==
                USBRQ_DIR_HOST_TO_DEVICE)
                sendEmptyFrame = 1;
        }
        
        //We dont handle any data from the host.
        return 0; //0xff;
    }
    
    /*---------------------------------------------------------------------------*/
    /* usbFunctionRead                                                           */
    /*---------------------------------------------------------------------------*/
    
    uchar usbFunctionRead(uchar * data, uchar len)
    {
        // DEBUG LED
        //PORTC ^= 0x02;
        
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        
        return 7;
    }
    
    /*---------------------------------------------------------------------------*/
    /* usbFunctionWrite                                                          */
    /*---------------------------------------------------------------------------*/
    
    uchar usbFunctionWrite(uchar * data, uchar len)
    {
        // DEBUG LED
        //PORTC ^= 0x04;
        return 1;
    }
    
    
    /*---------------------------------------------------------------------------*/
    /* usbFunctionWriteOut                                                       */
    /*                                                                           */
    /* this Function is called if a MIDI Out message (from PC) arrives.          */
    /*                                                                           */
    /*---------------------------------------------------------------------------*/
    
    void usbFunctionWriteOut(uchar * data, uchar len)
    {
        //TODO: Use midi callback for arduino to allow for listining.
        // DEBUG LED
        //PORTC ^= 0x20;
    }

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __DigiMIDI_h__
