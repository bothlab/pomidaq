#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Communication to DAQ Board definitions
#define RECORD_START		0x01
#define RECORD_END			0x02
#define SET_CMOS_SETTINGS	0x03

// USART Comm
#define GET_ID			0x01
#define GET_CURRENT0	0x30
#define GET_CURRENT1	0x31

#define SET_CURRENT		0x10
#define OSC_ON0			0x21
#define OSC_ON1			0x22
#define OSC_OFF			0x20

#endif // DEFINITIONS_H
