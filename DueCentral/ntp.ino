//*******************************************************************************
//*                                NTP support                                  *
//*     (the code is sort of standalone, thus all the globals are in here)      *
//*******************************************************************************

#define EthernetLedBlink() BlinkLed (LED_RED)

//
// Create an entry for each time server to use
// Define another array in PROGMEM for the time servers
//

prog_char ntp0[] PROGMEM = "ntp1.sul.t-online.de";
prog_char ntp1[] PROGMEM = "ntp6.space.net";
prog_char ntp2[] PROGMEM = "clock01.mnuk01.burstnet.eu";
prog_char *ntpList[] PROGMEM = { ntp0, ntp1, ntp2 };
static int currentTimeserver = 0; 
#define NUM_TIMESERVERS (sizeof (ntpList) / sizeof (ntpList[0]))

uint8_t clientPort = 123; 

//
// The next part is to deal with converting time received from NTP servers
// to a value that can be displayed. This code was taken from somewhere that
// I cant remember. Apologies for no acknowledgement.
//

uint32_t lastUpdate = 0;
uint32_t timeLong;
#define GETTIMEOFDAY_TO_NTP_OFFSET 2208988800UL

void DnsLookup (void) {
  EthernetLedBlink ();
  ether.dnsLookup ((char*) pgm_read_word (&(ntpList[currentTimeserver])));
} 

void UpdateTimeNtp (void) {
  uint16_t dat_p;
  int plen = 0;

  //
  // Main processing loop now we have our addresses
  // handle ping and wait for a tcp packet
  //

  plen = ether.packetReceive ();
  dat_p=ether.packetLoop (plen);
  
  //
  // Has unprocessed packet response
  //

  if (plen > 0) {
    timeLong = 0L;

    if (ether.ntpProcessAnswer (&timeLong, clientPort)) {
      if (timeLong) {
        EthernetLedBlink ();
        timeLong -= GETTIMEOFDAY_TO_NTP_OFFSET;
        setTime (CE.toLocal (timeLong)); 
      }
    }
  }

  //
  // Request an update every TIME_UPDATE_PERIOD seconds
  //

  if (((millis () - lastUpdate) >= (TIME_UPDATE_PERIOD * 1000UL)) 
     || (lastUpdate == 0)) {   // time to send request
    lastUpdate = millis();

    EthernetLedBlink ();
    if (ether.dnsLookup ((char*) pgm_read_word (&(ntpList[currentTimeserver])))) {
      ether.ntpRequest (ether.hisip, ++clientPort);
    }
    if (++currentTimeserver >= NUM_TIMESERVERS) {
      currentTimeserver = 0;
    }
  }
}

