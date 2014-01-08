// Find list of servers at http://support.ntp.org/bin/view/Servers/StratumTwoTimeServers
// Please observe server restrictions with regard to access to these servers.
// This number should match how many ntp time server strings we have
#define NUM_TIMESERVERS 3
static int currentTimeserver = 0;

// Create an entry for each timeserver to use
prog_char ntp0[] PROGMEM = "ntp1.sul.t-online.de";
prog_char ntp1[] PROGMEM = "ntp6.space.net";
prog_char ntp2[] PROGMEM = "clock01.mnuk01.burstnet.eu";

// Now define another array in PROGMEM for the above strings
prog_char *ntpList[] PROGMEM = { ntp0, ntp1, ntp2 };

uint8_t clientPort = 123;

// The next part is to deal with converting time received from NTP servers
// to a value that can be displayed. This code was taken from somewhere that
// I cant remember. Apologies for no acknowledgement.

uint32_t lastUpdate = 0;
uint32_t timeLong;
// Number of seconds between 1-Jan-1900 and 1-Jan-1970, unix time starts 1970
// and ntp time starts 1900.
#define GETTIMEOFDAY_TO_NTP_OFFSET 2208988800UL


void DnsLookup (void) {
  EthernetLedOn ();
  ether.dnsLookup( (char*)pgm_read_word(&(ntpList[currentTimeserver])) );
  EthernetLedOff ();
}


void UpdateTimeNtp (void) {
  uint16_t dat_p;
  int plen = 0;

  //
  // Main processing loop now we have our addresses
  // handle ping and wait for a tcp packet
  //

  plen = ether.packetReceive();
  dat_p=ether.packetLoop(plen);
  
  //
  // Has unprocessed packet response
  //

  if (plen > 0) {
    timeLong = 0L;

    EthernetLedOn();
    if (ether.ntpProcessAnswer(&timeLong,clientPort)) {
      DebugEthernetln( F( "Time has arrived" ));
      DebugEthernetln(timeLong); // secs since year 1900

      if (timeLong) {
        timeLong -= GETTIMEOFDAY_TO_NTP_OFFSET;
        setTime(CE.toLocal(timeLong));
        Flags |= F_TIME_UPDATED;
      }
    }
  }

  //
  // Request an update every TIME_UPDATE_PERIOD seconds
  //

  if( ((lastUpdate + TIME_UPDATE_PERIOD * 1000UL) < millis()) || (lastUpdate == 0)) {   // time to send request
    lastUpdate = millis();
    DebugEthernetln( F("DNS Lookup" ) );
    DebugEthernetln( currentTimeserver, DEC );

    if (ether.dnsLookup( (char*)pgm_read_word(&(ntpList[currentTimeserver])) )) {
      DebugEthernet( F("Send NTP request..." ));
      DebugEthernetln( currentTimeserver, DEC );
      ether.ntpRequest(ether.hisip, ++clientPort);
      DebugEthernetln( F("sent."));
    }
    if( ++currentTimeserver >= NUM_TIMESERVERS )
      currentTimeserver = 0;
  }
  EthernetLedOff ();
}

