#ifndef _TELEVISOR_
#define _TELEVISOR_
 
// *************************** Optimisation and Debug *************************************************************************
//#define  STUFFROMLOOP               // Stuff the buffer from main loop and not the interrupt
//#define  DEBUG_PLAYISR            // See how long the play isr takes, cannot use this at 44k
//#define  DEBUG_PIISR              // See how long the PI isr takes
//#define  DEBUG_ROUNDRROBIN        // See how fast the round robin is and how long the stuffer takes
//#define AUDIO_ONLY                // turns of the motor so you can test audio
//#define DEBUG_BUFFERSIZE          // shows how many bytes are being per block e.g. small numbers are good, use the serial plotter

// *************************** Arduino Library Support ************************************************************************
#include <Arduino.h>
//#define ROM const char PROGMEM /* adding progmem kills the string, why? */
#define ROM const char           /* need to work this out as ram usage drops 1774-1620=154 bytes! */

// *************************** SD Card Library Support ************************************************************************
#include <SdFat.h>
// Uses the SdFat Library at https://github.com/greiman/SdFat
// backup at http://www.taswegian.com/SdFat-master.zip

// *************************** Hardware details and settings  *****************************************************************
// Arduino pinout/connections:  https://www.taswegian.com/NBTV/forum/viewtopic.php?f=28&t=2298
//                              Pname  Description
#define PIN_MOTOR        3   /* D3     Motor PWM Pin D3 */
#define SD_CS_PIN        4   /* D4     Chip select for SD card */   
#define PIN_SOUND_ENABLE 5   /* D5     Enable sudio amp when high */
//                           /* D7     IR_IN, opto sensor - Analog comparator */
#define DEBUG_PIN       12   /* D12    Used for scoping performance */
//LED_BUILTIN           ??   /* D??    Built in red led, already defined by arduino.h */
// digitalWrite(DEBUG_PIN, HIGH);
//                           /* D13    Led PWM - PWM from TIMER4 */
// SD Card Clock             /* SCK    SD card uses this pin */
// SD Card Master In         /* MISO   SD card uses this pin */
// SD Card Master Out        /* MOSI   SD card uses this pin */
// To Nextion RX pin         /* TX1    Send serial to Nextion */
// To Nextion TX pin         /* RX1    Get serial from Nextion */

// Special pin defines
#define PWM187k 1   // 187500 Hz pwm frequency for TIMER4 on pin */

// *************************** CONFIGURATION... *******************************************************************************
#define SD_BLOCK_SIZE    63         // 64 is worse at high speed ,if many bytes are need to be loaded in the buffer how many bytes do we max it to?
//#define OVERRIDE_SPEED
//#define OVERRIDE_VALUE   25000
#define CIRCULAR_BUFFER_SIZE 256    /* must be a multiple of sample size, I think its the power of e.g. 128 256 512 245=14% of ram */
// (i.e, (video+audio)*bytes per)) = 4 for 16-bit, and 2 for 8-bit

// *************************** Functio0n prototypes ***************************************************************************
void commenceSelectedTrack(bool);
void writeMenuStrings(uint8_t base, uint8_t updateReq);

// *************************** Tunings PID Motor speed and sync ***************************************************************
#define KP_SPEED          1.0F      /* Speed proportional - this is how agressivly it moves towards and out of sycn frame - vertical */
#define KP_FRAME          0.2F      /* Frame proportional - this is how agressivly it moves towards and out of sycn frame - horizontal */
#define KI_SPEED          0.01F     /* Speed integral - slow/long term speed stability */
#define MAX_FRAME_ERR    50         /* if the PID error is smaller than this then frame locking will commence */
#define LARGE_SPEED_ERR  50         /* if speed error is large then turn off I (integral) */
#define MOT_PWM_MAX     150         // can set this lower to stop smaller power supploes tripping

// states - add proper state machine
// buttons can be signals that run with in states
// use enum here?
#define MODE_INIT             0
#define MODE_TITLE            1
#define MODE_TITLE_INIT       2
#define MODE_SELECT_TRACK     3
#define MODE_PLAY             4
#define MODE_SHIFTER          5
#define MODE_INFO_SCREEN      6

// *************************** Nextion "custom" defines  *********************************************************************
void gotTouch (struct nl_struct *ns);
void gotValue (struct nl_struct *ns);

#define PAGE_STRT     0              /* page 0 start up screen */
#define PAGE_MENU     1              /* page 1 select file */
#define PAGE_PLAY     2              /* page 2 playing screen */
#define PAGE_SHFT     3              /* page 3 shift */
#define PAGE_INFO     4              /* page 4 show info */

// page 1 events
#define H0_ID        11             /* seeker slider */
//#define H0_NAME     'h'
//ROM     H0_GET[] = {"h0"};

#define FIL_ID       1              /* file selected touch */
#define FIL_NAME    'f'             /* used to get the id of which file was selected */
ROM     FIL_GET[] = {"si"};

#define BAS_ID       14             /* base value for menu list */
#define BAS_NAME    'B'
ROM     BAS_GET[] = {"bx"};

#define RQU_ID       15             /* required update value for menu list */
#define RQU_NAME    'R'
ROM     RQU_GET[] = {"ru"};

// page 2 events
#define BRT_ID       5              /* brightness slider */
#define BRT_NAME    'b'
ROM     BRT_GET[] = {"b"};

#define CON_ID       7              /* contrast slider */
#define CON_NAME    'c'
ROM     CON_GET[] = {"c"};

#define VOL_ID       8              /* volume slider */
#define VOL_NAME    'v'
ROM     VOL_GET[] = {"v"};

#define SEK_ID       21             /* seeker slider */
#define SEK_NAME    's'
ROM     SEK_GET[] = {"s"};

#define GAM_ID       15             /* gamma / value */
#define GAM_NAME    'g'
ROM     GAM_GET[] = {"g"};

#define REP_ID       17             /* repeat / value */
#define REP_NAME    'r'
ROM     REP_GET[] = {"r"};

#define INF_ID       22             /* info button down */
#define SHF_ID       23             /* shift button down */
#define STP_ID       16             /* Stop button down */

// page 3 events
#define OKY_ID       7              /* okay button to exit this screen */
#define ADJ_ID       14             /* adjust button 10 bU up, 11 bL left, 12 bD down, 13 bR right */

#define XPO_ID       2              /* x position text */
#define XPO_NAME    'x'
ROM     XPO_GET[] = {"n0"}; // get x position

#define YPO_ID       5              /* y position text */
#define YPO_NAME    'y'
ROM     YPO_GET[] = {"n1"};         /* get y position */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "Magic" conversion of samples to modified video/audio values...

// The conversion tables 'audio[]' and 'video[]' may be reduced in size to save RAM. The size of each is
// independent, but both must be a power of two in size. The size is set by modifying the defines
// VIDEO_BITS and AUDIO_BITS. Each defining 2^N bytes of array size.  The code auto-configures based on
// these definitions, so that's  all you need to do.  In general, keep audio as big as possible, and video
// seems to be pretty good from 4 bits (16 brightnesses). Configuring VIDEO and AUDIO conversion array sizes:
// use values from 1 to 8 inclusive.

#define VIDEO_BITS    7                               /* giving 2^N levels */
#define AUDIO_BITS    8                               /* giving 2^N levels */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DO NOT MODIFY...  AUTO-CALCULATED!

#define VIDEO_SIZE ( 1 << VIDEO_BITS )                /* == 2^VIDEO_BITS */
#define AUDIO_SIZE ( 1 << AUDIO_BITS )                /* == 2^AUDIO_BITS */

#define VIDEO_SHIFT ( 8 - VIDEO_BITS )
#define AUDIO_SHIFT ( 8 - AUDIO_BITS )

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gamma correction using parabolic curve - Table courtesy Klaas Robers
const byte PROGMEM     gamma8[] = {
  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,
  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,
  5,  5,  6,  6,  6,  6,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17,
  17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25,
  26, 27, 27, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 36, 36,
  37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 46, 47, 47, 48, 49,
  50, 51, 52, 53, 54, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
  65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 80, 81,
  82, 83, 84, 85, 87, 88, 89, 90, 91, 92, 94, 95, 96, 97, 99,100,
  101,102,104,105,106,107,109,110,111,113,114,115,117,118,119,121,
  122,123,125,126,128,129,130,132,133,135,136,138,139,141,142,144,
  145,147,148,150,151,153,154,156,157,159,160,162,164,165,167,168,
  170,172,173,175,177,178,180,182,183,185,187,188,190,192,194,195,
  197,199,201,202,204,206,208,209,211,213,215,217,219,220,222,224,
  226,228,230,232,234,235,237,239,241,243,245,247,249,251,253,255
};

#endif

