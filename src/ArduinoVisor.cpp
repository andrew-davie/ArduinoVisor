////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino Mechanical Televisor  https://github.com/andrew-davie/NBTV                                                         //
// This program implements all the functions needed for a working mechanical televisor using an a                             //
// Arduino Micro. For more information see https://www.taswegian.com/NBTV/forum/viewforum.php?f=28                            //
//                                                                                                                            //
// program by Andrew Davie (andrew@taswegian.com), March 2017...                                                              //
// and Keith Colson - http://www.edns.co.nz/index.php/about/ - September 2017...                                               //
//                                                                                                                            //
// Notes:                                                                                                                     //
// 1. Strings in the code actually use RAM.  To save RAM, use the macro F("string")                                           //
// 2. >>>> Debug mode is entered by sending a character on the console within two seconds of startup <<<<                     //
// 3. using 70% ram or more can make the upload fail, hold the reset button down and release when upload starts to fix it     //
//                                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "televisor.h" // the main header file


void stopButtonCallback();
void fixVideoTable();
void fixAudioTable(uint8_t customVolume);
byte calculatePI(double speedErr, double frameErr);


// *************************** Nextion Library Support (touch screen) *********************************************************
#include "nexlite.h"   // file to get rid of nextion library

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool playing = false;
bool showInfo;               // info txt file present
bool debug = false;
bool nextion  = false;    // assume no nextion connected until found
bool forceTimePreventSeeker;

uint8_t uiMode; // = MODE_TITLE_INIT;     // program state
uint8_t selection = 0;       // selected track # from menu
uint8_t baseVal;
uint32_t lastSeekPosition;

uint32_t shiftFrame = 80;

// *************************** Global Variables ***************************************************************

// PID variables
double integral;
double lastTime;       // this is used to calculate delta time
double lastErr;        // this is used to calculate delta error
double motorSpeed;

File nbtv;
SdFat nbtvSD;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t video[VIDEO_SIZE];                            // convert sample to modified brightness/contrast/gamma/bit depth
uint8_t audio[AUDIO_SIZE];                            // convert sample to modified audio/bit depth

int16_t customBrightness;
long customContrast2;                                 // a X.Y fractional  (8 bit fractions) so 256 = 1.0f   and 512 = 2.0f

uint8_t customGamma;                                  // flags if gamma should be applied

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t customRepeat = 0;


uint8_t countFiles(void);
void resetStream(long seeker);
void setupFastPwm(int mode);
void setupMotorPWM();
void qInit(char *name);
void drawTime(const char *name, int sec);
void composeMenuItem(int item, int s, char *p, bool hunt);

uint8_t circularAudioVideoBuffer[CIRCULAR_BUFFER_SIZE];

uint32_t playbackAbsolute;
uint32_t lastPlaybackAbsolute;
uint32_t dataPosition;        // WAV file start of data chunk (for rewind/seek positioning)

uint8_t *pbp;

uint32_t videoLength;
uint32_t streamAbsolute;
uint16_t bufferOffset;
uint16_t sampleRate;
uint16_t singleFrame;
uint8_t bytesPerSample;               // bytes per sample
uint32_t oneSecond;

unsigned long lastDetectedIR;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void nl_Callback(struct nl_struct *ns) {// event from nextion - nl means nextion lite - ns means nextion struct
    //Serial.print(ns->buff[1]);  Serial.print("  ,  "); Serial.print(ns->buff[2]); Serial.print("  ,  "); Serial.println(ns->buff[3]);//  Serial.println(ns->buff[4]);
    switch (ns->buff[0]) {// command type
    case NEX_RET_EVENT_TOUCH_HEAD:      // touch happened       0x65 0x02 0x08 0x00 0xFF 0xFF 0xFF
        gotTouch(ns);                   // call the touch parser
        break;
    case NEX_RET_NUMBER_HEAD:           // get ?.val result     0x71 0x80 0x00 0x00 0x00 0xFF 0xFF 0xFF
        gotValue(ns);                   // call the number parser
        break;
    case NEX_RET_CMD_FINISHED:          // basically an ACK     0x01 0xFF 0xFF 0xFF
        break;                          // nothing needed here
    }
}

void titleCallback();

void gotTouch(struct nl_struct *ns) {

    switch (ns->buff[1]) {                // page number

    case PAGE_STRT:                       // page 0 has touch
        titleCallback();
        break;

    case PAGE_MENU:                       // page 1 has touch
        switch (ns->buff[2]) {            // id
        case H0_ID:                       // file slider touch
            //nl_GetVal(H0_GET, H0_NAME);   // get file list slide value
            nl_GetVal(BAS_GET, BAS_NAME); // get the base value (scroll position)        
            break;
        case FIL_ID:                      // file selected
            nl_GetVal(FIL_GET, FIL_NAME); // get file to open slide value
            break;
        }
        break;

    case PAGE_PLAY:                       // page 2 has touch
        switch (ns->buff[2]) {            // id
        case BRT_ID:                      // brightness slider touch
            nl_GetVal(BRT_GET, BRT_NAME); // get brightness slide value
            break;
        case CON_ID:                      // contrast slider touch
            nl_GetVal(CON_GET, CON_NAME); // get contrast slide value
            break;
        case VOL_ID:                      // 0x71 0x80 0x00 0x00 0x00 0xFF 0xFF 0xFF
            nl_GetVal(VOL_GET, VOL_NAME); // get volume slide value
            break;
        case SEK_ID:                      // seeker slider touch
            nl_GetVal(SEK_GET, SEK_NAME); // get seeker slide value
            break;
        case INF_ID:                      // info button touch down
            if (showInfo) {
                nl_SetPage(PAGE_INFO);    // show info page
                uiMode = MODE_INFO_SCREEN;// set mode as info page
            }
            break;
        case SHF_ID:                      // shift button touch down
            nl_SetPage(PAGE_SHFT);        // show shift page
            uiMode = MODE_SHIFTER;
            break;
        case GAM_ID:                      // gamma button touch down
            nl_GetVal(GAM_GET, GAM_NAME); // get gamma button value
            break;
        case REP_ID:
            nl_GetVal(REP_GET, REP_NAME); // get repeat button value
            break;
        case STP_ID:                      // stop button touch down
            stopButtonCallback();
            break;
        }
        break;

    case PAGE_SHFT:                       // Shifter page 3 event
        switch (ns->buff[2]) {            // id
        case OKY_ID:                      // okay button to exit this screen
            nl_SetPage(PAGE_PLAY);
            uiMode = MODE_PLAY;
            playing = true;
            break;
        case ADJ_ID:                      // adjust button 10 bU up, 11 bL left, 12 bD down, 13 bR right
            // can get x here and get y in the response to x arriving
            nl_GetVal(XPO_GET, XPO_NAME); // get x shift value
            //nl_GetVal(YPO_GET,YPO_GET); // get y shift value
            break;
        }
        break;

    case PAGE_INFO:                       // Info page 4 touch event
        nl_SetPage(PAGE_PLAY);            // Go back to the play screen
        break;
    }
}

void gotValue (struct nl_struct *ns) {  // 0x71 0x80 0x00 0x00 0x00 0xFF 0xFF 0xFF
    //Serial.print("prevId = ");Serial.write(ns->prevId);Serial.print(", val = ");Serial.println(ns->buff[1]);
    switch (ns->prevId) {
    case BRT_NAME:                      // value of brightness control slider
        customBrightness = ( ns->buff[1] - 128 ); // << 1;
        fixVideoTable();
        break;
    case CON_NAME:                      // value of contrast control slider
        customContrast2 = ns->buff[1] * 2; // Serial.print("Contraat Val = "); Serial.println(ns->buff[1]);
        fixVideoTable();
        break;
    case VOL_NAME:                      // value of volume control slider
        fixAudioTable(ns->buff[1]);
        break;
    case GAM_NAME:                      // gamma toggle button value
        customGamma = ns->buff[1];
        fixVideoTable();
        break;
    case REP_NAME:                      // repeat toggle button value
        customRepeat = ns->buff[1];
        break;
    case SEK_NAME:                      // value of seeker control slider
        uint32_t seekPos;
        seekPos = (long) ( ( (double) ns->buff[1] / 255. ) * ( (double) videoLength / singleFrame ) );
        resetStream(seekPos * singleFrame);
        forceTimePreventSeeker = true;        // draw the time but not the seeker bar
        break;
    case FIL_NAME:                      // value of file selected
        selection = ns->buff[1];        // track selected
        commenceSelectedTrack(true);
        break;
    case BAS_NAME:                      // got the base position of the file scroll list
        baseVal = ns->buff[1];
        nl_GetVal(RQU_GET, RQU_NAME);   // what kind of update needs doing to the file list screen?
        break;
    case RQU_NAME:                      // file list needs updating 
        writeMenuStrings(baseVal, ns->buff[1]);
        break;        
    case XPO_NAME:                      // value of shifter x position
        Serial.print("shifter x = "); Serial.println(ns->buff[1]);
        //shiftFrame = ns->buff[1];
        //NexVariable shift = NexVariable(3, 1, "s");
        //shift.getValue(&shiftFrame);
        nl_GetVal(YPO_GET, YPO_NAME);   // get y shift value
        break;
    case YPO_NAME:                      // value of shifter x position
        Serial.print("shifter y = "); Serial.println(ns->buff[1]);
        //shiftFrame = ns->buff[1];
        //NexVariable shift = NexVariable(3, 1, "s");
        //shift.getValue(&shiftFrame);
        break;
    case 9:                      //  what was this? maybe the file scroll bar?
        Serial.print("wtf = "); Serial.println(ns->buff[1]);
        break;
    }
}

void fixVideoTable() {
    for (int i = 0; i < VIDEO_SIZE; i++) {

        long val = ( i << VIDEO_SHIFT ) * customContrast2;
        val >>= 8;
        val += customBrightness;

        if (val < 0)
            val = 0;
        if (val > 255)
            val = 255;
        if (customGamma)
            val = pgm_read_byte(&gamma8[val]);   // for now we have kept the gamma table
        video[i] = val;
    }
}

void fixAudioTable(uint8_t customVolume) {
    uint8_t logVolume = pgm_read_byte(&gamma8[customVolume]);
    for (int i = 0; i < AUDIO_SIZE; i++) {
        audio[i] = ( ( (int) ( ( i << AUDIO_SHIFT ) - 0x80 ) ) * logVolume + 0x8000 ) >> 8;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Data playback interrupt
// This is an interrupt that runs at the frequency of the WAV file data (i.e, 22050Hz, 44100Hz, ...)
// It takes data from the circular buffer 'circularAudioVideoBuffer' and sends it to the LED array
// and to the speaker.

#ifndef STUFFROMLOOP
bool alreadyStreaming = false;
#endif

ISR(TIMER3_OVF_vect) {

    OCR4A = video[*pbp++
#if VIDEO_SHIFT > 0
                  >> VIDEO_SHIFT
 #endif
            ];

    TCCR4A = 0x82;

    // Write the audio to pin 6 PWM duty cycle
    OCR4D = audio[*pbp++
#if AUDIO_SHIFT > 0
                  >> AUDIO_SHIFT
#endif
            ];


    DDRD |= 0x80;
    TCCR4C |= 0x09;

    playbackAbsolute += bytesPerSample;

    if (pbp >= circularAudioVideoBuffer + CIRCULAR_BUFFER_SIZE)
        pbp = circularAudioVideoBuffer;

#ifdef STUFFROMLOOP  // stuffing from loop and this runs 8.4khz round robin
}

void stuffer() {
#endif


    // This code tries to fill up the unused part(s) of the circular buffer that contains the streaming
    // audio & video from the WAV file being played. 'bufferOffset' is the location of the next buffer
    // write, and this wraps when it gets to the end of the buffer. The number of bytes to write is
    // calculated from two playback pointers - 'playbackAbsolute' which is the current position in the
    // audio/video stream that's ACTUALLY being shown/heard, and 'streamAbsolute' which is the position
    // of the actually streamed data (less 1 actual buffer length, as it's pre-read at the start). Those
    // give us a way to calculate the amount of free bytes ('bytesToStream') which need to be filled by
    // reading data from the SD card. Note that 'playbackPointer' can (and will!) change while this
    // routine is streaming data from the SD.  The hope is that we can read data from the SD to the buffer
    // fast enough to keep the playback happy. If we can't - then we get glitches on the screen/audio

#ifndef STUFFROMLOOP
    if (!alreadyStreaming) {
        alreadyStreaming = true;                      // prevent THIS *part of the code* from interrupting itself...
        sei();                                        // allow interrupt again
#endif

    uint32_t bytesToStream = playbackAbsolute - streamAbsolute;
    if (bytesToStream > 63) {                         // might be more efficient reading larger blocks!

        void *dest = (void *) ( circularAudioVideoBuffer + bufferOffset );
        bufferOffset += bytesToStream;
        if ( bufferOffset >= CIRCULAR_BUFFER_SIZE ) {
            bytesToStream = CIRCULAR_BUFFER_SIZE - bufferOffset + bytesToStream;
            bufferOffset = 0;
        }
        nbtv.read( dest, bytesToStream );
        streamAbsolute += bytesToStream;
    }

#ifndef STUFFROMLOOP
    alreadyStreaming = false;
}
#endif

}

//-- Analog comparator interrupt service routine -------------------------------------------------
// Triggered by sync hole detected by IR sensor, connected to pin 7 (AC)

ISR(ANALOG_COMP_vect)
{
    //Serial.println(F("IR"));
    uint32_t irAbsolute = playbackAbsolute;

    double deltaSample = irAbsolute - lastDetectedIR;
    // Serial.println(deltaSample); always = 3072 to 3074 when synced

    if (deltaSample < 1000)                                                 //debounce or stopped
        return;

    // should really off load all the calcs to main and keep this super lean
    lastDetectedIR = irAbsolute; // store for use on next cycle

    // This next bit is super cool.  The PID synchronises the disc speed to 12.5 Hz (indirectly through the singleFrame
    // value, which is a count of #samples per rotation at 12.5 Hz).  Since the disc and the playback of the video
    // are supposed to be exactly in synch, we know that both SHOULD start a frame/rotation at exactly the same time.
    // So since we know we're at the start of a rotation (we just detected the synch hole), then we can compare
    // how 'out of phase' the video is by simply looking at the video playback sample # ('plabackAbsolute') and
    // use the sub-frame offset as an indicator of how inaccurate the framing is.  Once we know that, instead of
    // trying to change the video timing, we just change the disc speed. How do we do that?  By tricking the PID
    // into thinking the disc is running slower than it really is by telling it that the time between now and
    // the last detected rotation is actually longer than measured. A consequence of this is that the PID will try
    // to speed the disc up a fraction, which will in turn mean the video is playing slightly slower relative
    // to the timing of the disc, and the image will shift left and up.  We also (gosh this is elegant) get the
    // ability to shift the image up/down.  So we can hardwire the exact framing vertical of the displayed image,
    // too.

    int32_t pbDelta = irAbsolute % singleFrame;                       // how inaccurate is framing?

    if (pbDelta >= singleFrame / 2) // its faster to re frame in the other direction
        pbDelta = -( singleFrame - pbDelta ); // this needs to be a negative number to get the other direction
    //Serial.println(pbDelta);  // max about 3100 nominal about 75 shiftFrame = 75 so thats the matching number!

#define P_BODGE 0 /*7 for keith*/

    double sError = ( deltaSample - ( 3072 - P_BODGE ) ); // spead error - proportional with target of 3072 - bodge to get P perfectly balanced
    double fError = ( pbDelta - 75 /*+ 40*/ );        // frame error

    byte pidx = calculatePI(sError, fError);               // write the new motor PI speed

    if (pidx > 0) // if it is a good return value
        //Serial.println(pidx);
        OCR0B = pidx; // set motor pwm value
        
    #ifdef DEBUG_PIISR
      digitalWrite(DEBUG_PIN, LOW);          // round robin speed
    #endif          
}

// PID (from http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
byte calculatePI(double speedErr, double frameErr) { // pass in error
    double now = ( (double) playbackAbsolute ) / singleFrame;
    double timeChange = now - lastTime;
    double proportional;
    float motorPWM; 

    if (lastTime == 0) {                 // avoids glitch on first round
        lastTime = now;
        return 0;                        // return zero for error (no motor action will be taken)
    }
    proportional = KP_SPEED * speedErr;  // do this multiplication just once

    integral += ( frameErr * timeChange ); // this is the integral value

    if (speedErr > LARGE_SPEED_ERR)      // if speed error is large then turn off integral
        integral = 0;                    // reset the integral

    if (speedErr < MAX_FRAME_ERR)        // if speed error is small then tune the frame
        proportional = proportional + ( frameErr * KP_FRAME ); // this is how aggressive it hunts the frame edge
    motorPWM =  (proportional + ( KI_SPEED * integral )) + 0.5;    
    if (motorPWM > MOT_PWM_MAX)          // limit pwm to avoid overcurrent
        motorPWM = MOT_PWM_MAX;
    if (motorPWM < 1)                    // if smaller than 1
        motorPWM = 1;                    // return 1 for no error

    lastErr = speedErr;                  // store globals for next round
    lastTime = now;
    return (uint8_t) motorPWM;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PWM to control motor speed

void setupMotorPWM() {

    pinMode( PIN_MOTOR, OUTPUT );

    // If we're using an IRL540 MOSFET for driving the motor, then it's possible for the floating pin to cause
    // the motor to spin. So we make sure the pin is set LOW ASAP.
    digitalWrite( PIN_MOTOR, LOW );

    // The timer runs as a normal PWM with two outputs but we just use one for the motor
    // The prescalar is set to /1, giving a frequency of 16000000/256 = 62500 Hz
    // The motor will run off PIN 3 ("MOTOR_DUTY")
    // see pp.133 ATmega32U4 data sheet for WGM table.  %101 = fast PWM, 8-bit

    TCCR0A = 0
             | bit(COM0A1)                          // COM %10 = non-inverted
             | bit(COM0B1)                          // COM %10 = non-inverted
             | bit(WGM01)
             | bit(WGM00)
    ;

    TCCR0B = 0
             //| bit(WGM02)
             | bit(CS02)                           // prescalar %010 = /1 = 16000000/256/1 = 62.5K
             //| bit(CS00)
    ;

    OCR0B = MOT_PWM_MAX;                            // set motor pwm full throttle!
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void play(char *filename, unsigned long seekPoint = 0);
bool getFileN(int n, int s, char *name, bool reset, bool strip);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// The callback "happens" when there is a change in the scroll position of the dialog.  This
// means that we need to update one or all of the 8 text lines with new contents. When the up
// or down arrows are pressed, the Nextion automatically does the scrolling up/down for the
// lines that are visible on the screen already, and we only need to update the top (0 for up)
// or the bottom (7 for down) line.  However, if the slider was used, we have to update all
// of the lines (8) - so this is somewhat slower in updating.  Not too bad though.

void refresh(int base, int i) {
    char nx[64];
    strcpy(nx, "f0.txt=\"");
    nx[1] = '0' + i;
    composeMenuItem(base + i, sizeof( nx ) - 8, nx + 8, true);
    strcat(nx, "\"");
    nl_SendCommand2(nx);
}

#define REFRESH_TOP_LINE 0
#define REFRESH_BOTTOM_LINE 7
#define REFRESH_ALL_LINES 9
void writeMenuStrings(uint8_t base, uint8_t updateReq) {

    // if (debug) {
    //   Serial.print(F("Base="));
    //   Serial.print(base);
    //   Serial.print(F("   uReq="));
    //   Serial.println(updateReq);
    // }

    switch (updateReq){
    case REFRESH_TOP_LINE:
    case REFRESH_BOTTOM_LINE:
      refresh(base, updateReq);
      break;    
    default://REFRESH_ALL_LINES
        for (int i = 0; i < 8; i++){
            refresh(base, i);
        }        
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void prepareControlPage() {

    // Fill in the one-time-only objects on the page

    char nx[16];

    sprintf(nx, "g.val=%d", customGamma);
    nl_SendCommand2(nx);
    sprintf(nx, "r.val=%d", (int) customRepeat);
    nl_SendCommand2(nx);

    // Set "i" button ORANGE if there's no text
    sprintf(nx, "q.pic=%d", showInfo ? 13 : 20);
    nl_SendCommand2(nx);
    sprintf(nx, "q.pic2=%d", showInfo ? 15 : 20);
    nl_SendCommand2(nx);

    drawTime("tmax", videoLength / oneSecond );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void commenceSelectedTrack(bool firstTime = true) {

    char nx2[64];
    strcpy(nx2, "stn.txt=\"");
    char *nx = nx2 + 9;

    if (debug) {
        Serial.print(F("Start "));
        Serial.print(selection);
    }
      


    // Restart from the beginning once the no-nextion version runs out of tracks to play
    if (!nextion && !getFileN(selection, sizeof( nx2 ) - 9, nx, true, false)) {
        if (debug) {
            Serial.println(F("zap!"));
        }
        selection = 0;
    }

    if (getFileN(selection, sizeof( nx2 ) - 9, nx, true, false)) {

        if (firstTime) {
            customRepeat = false;                                       // 0 on new track, or keep existing if repeating
            qInit(nx);                                                  // read and buffer INFO lines --> nextion
        }


        lastPlaybackAbsolute = -oneSecond;
        lastSeekPosition = -1;

        setupMotorPWM();  // turn motor on at speed to spool up

        setupFastPwm(PWM187k); // get pwm ready for light source

        play(nx);

        digitalWrite(PIN_SOUND_ENABLE, HIGH);                                                                                   // enable amplifier



        // Strip the extension off the file name and send the result (title of the track) to the nextion
        // stored in the stn (stored track name) variable. This is loaded by the Nextion at start of page 2 display

        if (nextion) {
            strcpy(strstr(nx, ".wav"), "\"");
            nl_SendCommand2(nx2);                                        // --> stn.txt="track name"
        }

        playbackAbsolute = 0;                                                                         //?  TODO: probably removed -was just there to test/fix standalone playback
        customBrightness = 0;
        customContrast2 = 0x100;
        customGamma = true;

        fixVideoTable();
        fixAudioTable(128);


        uiMode = MODE_PLAY;
        playing = true;            

        if (firstTime) {
            if (nextion) {
                nl_SetPage(PAGE_PLAY);
                prepareControlPage();
            }
        }

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stopButtonCallback(void) {

    digitalWrite(PIN_SOUND_ENABLE, LOW);      // disable amplifier

    TCCR4B = 0;                                       // effectively turn off LED interrupt
    TCCR3A = 0;
    TCCR3B = 0;                                       // turn off motor buffer stuffer and playback
    TCCR0A = 0;                                       // motor PWM OFF
    OCR0A  = 0;                                       // motor OFF (just in case PWM had a spike)

    nbtv.close();                                     // close any previously playing/streaming file

    OCR4A = 0;                                        // blacken LED (=screen)
    DDRC |= 1 << 7;                                   // Set Output Mode C7
    TCCR4A = 0x82;                                    // Activate channel A

    uiMode = nextion ? MODE_TITLE_INIT : MODE_INIT;
    playing = false;    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void qInit(char *name) {

    // Copy video description (if any) from text file to the global variables on the
    // menu page. These are later copied to the description (Q) page lines on init of
    // that page by the Nextion itself. We do it here as globals so that we don't need
    // to flip Nextion pages, and we get auto-init on page draw.

    // Drop off the '.wav' and append '.txt' to give us the description filename
    char nx[64];
    strcpy(nx, name);
    strcpy(strstr(nx, ".wav"), ".txt");

    // grab the video details and write to the Q page
    File explain = nbtvSD.open(nx);
    showInfo = explain;

    for (int i = 0; i < 12; i++) {
        sprintf(nx, "I.t%d.txt=\"", i);
        char *p;
        for (p = nx + 10; *p; p++) {}
        if (explain) {
            do {
                explain.read(p, 1);
            } while (*p++ != 10);
            p--;
        }
        strcpy(p, "\"");
        if (nextion)
            nl_SendCommand2(nx);
    }
    explain.close();
}


void titleCallback() {
    //nl_SendCommand2(SHOW_MENU);                          // --> MENU
    nl_SetPage(PAGE_MENU);

    // Count the number of menu items and then set the maximum range for the slider
    // We subtract 8 from the count because there are 9 lines already visible in the window

    uint8_t menuSize = countFiles();
//  if (debug)
//    Serial.println(menuSize);

    char nx[16];
    sprintf(nx, "h0.maxval=%d", menuSize > 9 ? menuSize - 8 : 0);
    nl_SendCommand2(nx);

    writeMenuStrings(0,9);                             // defaults to REFRESH_ALL_LINES, so screen is populated
    uiMode = MODE_INIT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The Arduino starts with a one-time call to setup(), so this is where the Televisor is initialised.
// That's followed by calls to loop() whenever there is free time.

void setup() {
    // Turn OFF the sound amp
    pinMode(PIN_SOUND_ENABLE, OUTPUT);
    digitalWrite(PIN_SOUND_ENABLE, LOW);

    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // red led off

    Serial.begin(115200);// why slow it down to 9600?

    // IMPORTANT: DEBUG MODE ENTERED BY SENDING CHARACTER TO ARDUINO VIA CONSOLE IN 1st 2 SECONDS

    // Determine if there is debugging is required by looking for the presence of a character in the
    // serial buffer. Thus, we don't "hang" the code if there's no serial port.  It won't matter if we
    // actually write to the port if there's nothing there.

    while (!Serial.available() && millis() < 2000) ;

    if (Serial.available()) {
        while (Serial.available())
            Serial.read();
        Serial.println(F("DEBUG"));
        debug = true;
    }

    //debug = true;                         //tmp


    // Setup access to the SD card
    pinMode(SS, OUTPUT);
    if (!nbtvSD.begin(SD_CS_PIN))
        if (debug)
            Serial.println(F("SD failed!"));

    nextion = nl_Init(&nl_Callback);    // find out if nextion connected and setup call back for touch events

    uiMode = nextion ? MODE_TITLE_INIT : MODE_INIT;


    if (debug) {
        if (!nextion)
            Serial.print(F("NO "));
        Serial.println(F("display!"));
    }
        
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t countFiles() {

    char name[64];

    uint8_t count = 0;
    FatFile *vwd = nbtvSD.vwd();
    vwd->rewind();

    SdFile file;
    while (file.openNext(vwd, O_READ)) {
        file.getName(name, sizeof( name ) - 1);
        file.close();
        if (*name != '.' && strstr(name, ".wav"))
            count++;
    }

    return count;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get a filename from the SD card.  Hidden (starting with ".") files are ignored. Any file with
// a ".wav" in the filename is considered a candidate. Function fills a character pointer with the
// filename.
// Parameters
// hunt - true: start from the beginning of the SD file list and return the "nth" file
//       false: just return the very next files

bool getFileN(int n, int s, char *name, bool hunt = true, bool strip = true) {

    FatFile *vwd = nbtvSD.vwd();
    if (hunt)
        vwd->rewind();

    SdFile file;
    while (file.openNext(vwd, O_READ)) {
        file.getName(name, s);
        file.close();
        if (*name != '.') {
            char *px = strstr(name, ".wav");
            if (px) {
                if (strip)
                    *px = 0;
                if (!hunt || ( hunt && !n-- ))
                    return true;
            }
        }
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void composeMenuItem(int item, int s, char *p, bool hunt) {
    sprintf(p, "%2d ", item + 1);
    if (!getFileN(item, s - 3, p + 3, hunt))
        *p = 0;
}


void resetStream(long seeker) {

    TIMSK3 &= ~_BV(TOIE3);                            // DISABLE playback interrupt
    digitalWrite(PIN_SOUND_ENABLE, LOW);              // disable amplifier

    nbtv.seek(dataPosition + seeker);                 // seek to new position

    streamAbsolute = playbackAbsolute = seeker;
    lastDetectedIR = playbackAbsolute - singleFrame + shiftFrame;
    lastTime = ( (double) playbackAbsolute ) / singleFrame;     // keep the PID happy

    pbp = circularAudioVideoBuffer;
    bufferOffset = 0;

    nbtv.read((void *) circularAudioVideoBuffer, CIRCULAR_BUFFER_SIZE);   // pre-fill the circular buffer so it's valid

    digitalWrite(PIN_SOUND_ENABLE, HIGH);             // enable amplifier

    TIMSK3 |= _BV(TOIE3);                             // ENABLE playback interrupt
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Super dooper high speed PWM (187.5 kHz) using timer 4, used for the LED (pin 13) and audio (pin 6). Timer 4 uses a PLL as
// input so that its clock frequency can be up to 96 MHz on standard Arduino Leonardo/Micro. We limit imput frequency to 48 MHz
// to generate 187.5 kHz PWM. Can double to 375 kHz. ref: http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html

void setupFastPwm(int mode) {

    TCCR4A = 0;
    TCCR4B = mode;
    TCCR4C = 0;
    TCCR4D = 0;                                       // normal waveform counting up to TOP (in OCR4C) - i.e., 255
    PLLFRQ = ( PLLFRQ & 0xCF ) | 0x30;                         // Setup PLL, and use 96MHz / 2 = 48MHz
    OCR4C = 255;                                      // Target count for Timer 4 PWM
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Comparator interrupt for IR sensing
// precludes timer 1 usage

void setupIRComparator() {

    ACSR &= ~(
        bit(ACIE)                                                 // disable interrupts on AC
        | bit(ACD)                                                 // switch on the AC
        | bit(ACIS0)                                                 // falling trigger
        );

    ACSR |=
        bit(ACIS1)
        | bit(ACIE)                                                 // re-enable interrupts on AC
    ;

    SREG |= bit(SREG_I);                          // GLOBALLY enable interrupts
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool wavInfo(char *filename) {

    nbtv = nbtvSD.open(filename);
    if (!nbtv)
        return false;

    uint32_t header;
    // this should read into a struct
    nbtv.read(&header, 4);
    if (header != 0x46464952)                           //'FFIR'
        return false;

    long chunkSize;
    nbtv.read(&chunkSize, 4);

    nbtv.read(&header, 4);
    if (header != 0x45564157)                           //'EVAW'
        return false;

    while (true) {

        nbtv.read(&header, 4);                                                 // read next chunk header

        if (header == 0x7674626e) {                                                 //'vtbn'
            nbtv.read(&chunkSize, 4);
            unsigned long position = nbtv.position();
            nbtv.seek(position + chunkSize);
            continue;
        }

        else if (header == 0x20746D66) {                                                 //' tmf'

            nbtv.read(&chunkSize, 4);
            unsigned long position = nbtv.position();

            int audioFormat;
            nbtv.read(&audioFormat, 2);

            int numChannels;
            nbtv.read(&numChannels, 2);

            nbtv.read(&sampleRate, 4);

            long byteRate;
            nbtv.read(&byteRate, 4);

            int blockAlign;
            nbtv.read(&blockAlign, 2);

            unsigned int bitsPerSample;
            nbtv.read((void *) &bitsPerSample, 2);

            bytesPerSample = bitsPerSample >> 2;         // audio+video accounted for

            oneSecond = sampleRate * bytesPerSample;
            singleFrame = oneSecond / 12.5;             // todo: derive frames per second instead of hardwire


            // Potential "ExtraParamSize/ExtraParams" ignored because PCM

            nbtv.seek(position + chunkSize);
            continue;
        }

        else if (header == 0x61746164) {                                                 //'atad'
            nbtv.read(&videoLength, 4);
            dataPosition = nbtv.position();
            // and now the file pointer should be pointing at actual sound data - we can return

            break;
        }

        return false;
    }

    return true;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void play(char *filename, unsigned long seekPoint) {

    if (debug)
        Serial.println(filename);

    if (wavInfo(filename)) {

        lastErr   = 0;
        lastTime  = 0;
        integral  = 0;

        SREG &= ~bit(SREG_I);                                                 // GLOBALLY disable interrupts

        long seeker = singleFrame * seekPoint * 12.5;
        resetStream(seeker);

        // Uses /1 divisor, but counting up to 'resolution' each time  --> frequency!
        // This is timer 3, which is handling the playback of video and audio

        ICR3 = (int) ( ( 16000000. / sampleRate ) + 0.5 );                                    // playback frequency - input Capture Register 3

        TCCR3A =
            _BV(WGM31)                                                                         // Fast PWM (n1)   --> n3n2n1 == 111 --> Fast PWM
            | _BV(COM3A1)                                                                         // compare output mode = nA1nA0 = 10 --> Clear OCnA/OCnB/OCnC on compare match
                                                                                                  // (set output to low level); match is when the counter == ICR3
        ;

        TCCR3B =
            _BV(WGM33)                                                                         // Fast PWM (n3) see above
            | _BV(WGM32)                                                                         // Fast PWM (n2) see above
            | _BV(CS30)                                                                         // Clock select = /1 (no scaling) see pp.133 ATmega32U4 manual for table
        ;

        TIMSK3 |= (                                                                         // Timer3Interrupt Mask Register
            0
            | _BV(TOIE3)                                                                         // ENABLE timer overflow interrupt enable
            );

        SREG |= bit(SREG_I);                                                 // GLOBALLY enable interrupts
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void drawTime(const char *name, int sec) {

    int s = sec % 60;
    int m = sec / 60;

    char cmd[32];
    sprintf(cmd, "%s.txt=\"%2d:%02d\"", name, m, s);
    nl_SendCommand2(cmd);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Check to see if the data for the video has run out
// If so, then we stop the video, the motor, and then handle possible repeat button.
// and remain in the same (sub) mode we might already have been in (INFO or SHIFT, for example)

void handleEndOfVideo(int nextMode) {

    if (playbackAbsolute >= videoLength) {                                // Check for end of movie

        if (debug) {
            Serial.print(F("end track"));
        }

        if (!nextion)
            selection++;                                         // if there's no nextion/UI, cycle to next track

        stopButtonCallback();                                   // stop everyting (same as 'pressing' stop button)

        if (!nextion || customRepeat) {                           // BUT we might have the repeat button ON
            if (debug) {
                Serial.print(F("repeating"));
            }
            commenceSelectedTrack(false);                          // in which case we restart the track (NOT first time)
            uiMode = nextMode;                                     // and drop back into the previous UI mode
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

    if (nextion)
        nl_Listen(); // read nextion!

    if (playing){

        #ifdef DEBUG_ROUNDRROBIN
          digitalWrite(DEBUG_PIN, HIGH);          // round robin speed
        #endif  
        
        #ifdef STUFFROMLOOP
          stuffer(); // these is a better place to call this as it should happen when info screen is shown while playing etc
        #endif       // this will do for testing

        #ifdef DEBUG_ROUNDRROBIN
          digitalWrite(DEBUG_PIN, LOW);           // 4.5Khz ISR mode, STUFFROMLOOP 8.4Khz
        #endif

        // Display elapsed time in format MM:SS
        if (nextion){
          if ( forceTimePreventSeeker || playbackAbsolute - lastPlaybackAbsolute >= oneSecond ) {   // only update if > 1 second difference
              lastPlaybackAbsolute = playbackAbsolute;
              
              #ifdef DEBUG_BUFFERSIZE
                Serial.println(playbackAbsolute - streamAbsolute);// this will be broken since the merge
              #endif
              
              drawTime("timePos", playbackAbsolute / oneSecond);

              if (!forceTimePreventSeeker) {
          
                  // Adjust the seekbar position to the current playback position
                  uint32_t seekPosition = 256. * playbackAbsolute / videoLength;
                  //uint32_t seekPosition = ( playbackAbsolute << 8 ) / videoLength;
                  if (seekPosition != lastSeekPosition) {
                      lastSeekPosition = seekPosition;
                      char cmd[] = "s.val=XXX";
                      utoa(seekPosition, cmd + 6, 10);
                      nl_SendCommand2(cmd);
                  }
              }
          
              forceTimePreventSeeker = false;
          }
       }

       handleEndOfVideo(MODE_PLAY);        
        
    }else

    switch (uiMode) {

        case MODE_TITLE_INIT:
            if (nextion) {
                nl_SendCommand2("bkcmd=2");  // TODO: not needed?
                nl_SendCommand2("page 0");
            }
            uiMode = MODE_TITLE;
            break;

        case MODE_INIT:
            setupIRComparator();                          // enables the comparator
            uiMode = MODE_SELECT_TRACK;
            break;

        case MODE_SELECT_TRACK:                               // file selection from menu
            if (!nextion) {
                selection = 0;
                commenceSelectedTrack(true);
                customRepeat = true;                                                                         // in this case, play all tracks and repeat
            }
            break;
    }
}

// EOF

