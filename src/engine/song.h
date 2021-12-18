#include <stdio.h>
#include <vector>
#include "../ta-utils.h"
#include "orders.h"
#include "instrument.h"
#include "pattern.h"
#include "wavetable.h"
#include "sample.h"

enum DivSystem {
  DIV_SYSTEM_NULL=0,
  DIV_SYSTEM_YMU759,
  DIV_SYSTEM_GENESIS,
  DIV_SYSTEM_GENESIS_EXT,
  DIV_SYSTEM_SMS,
  DIV_SYSTEM_GB,
  DIV_SYSTEM_PCE,
  DIV_SYSTEM_NES,
  DIV_SYSTEM_C64_6581,
  DIV_SYSTEM_C64_8580,
  DIV_SYSTEM_ARCADE,
  DIV_SYSTEM_YM2610,
  DIV_SYSTEM_YM2610_EXT
};

struct DivSong {
  // version number used for saving the song.
  // Furnace will save using the latest possible version,
  // but eventually I will and 0x80 to this value to indicate a Furnace module
  // known version numbers:
  // - 24: v0.12/0.13/1.0
  //   - current format version
  //   - changes pattern length from char to int, probably to allow for size 256
  // - 23: ???
  //   - what happened here?
  // - 20: v11.1 (?)
  //   - E5xx effect range is now ±1 semitone
  // - 19: v11
  //   - introduces Arcade system
  //   - changes to the FM instrument format due to YMU759 being dropped
  // - 18: v10
  //   - radically changes STD instrument for Game Boy
  // - 17: v9
  //   - changes C64 volIsCutoff flag from int to char for unknown reasons
  // - 16: v8 (?)
  //   - introduces C64 system
  // - 15: v7 (?)
  // - 14: v6 (?)
  //   - introduces NES system
  //   - changes macro and wave values from char to int
  // - 13: v5.1
  //   - introduces PC Engine system in later version (how?)
  //   - stores highlight in file
  // - 12: v5 (?)
  //   - introduces Game Boy system
  //   - introduces wavetables
  // - 11: ???
  //   - introduces Sega Master System
  //   - custom Hz support
  //   - instrument type (FM/STD) present
  //   - prior to this version the instrument type depended on the system
  // - 10: ???
  //   - introduces multiple effect columns
  // - 9: v3.9
  //   - introduces Genesis system
  //   - introduces system number
  // - 7: ???
  // - 5: BETA 3 (?)
  //   - adds arpeggio tick
  // - 3: BETA 2
  //   - possibly the first version that could save
  //   - basic format, no system number, 16 instruments, one speed, YMU759-only
  //   - if somebody manages to find a version 2 or even 1 module, please tell me as it will be worth more than a luxury vehicle
  unsigned char version;

  // system
  DivSystem system;

  // song information
  String name, author;

  // legacy song information
  String carrier, composer, vendor, category, writer, arranger, copyright, manGroup, manInfo, createdDate, revisionDate;

  // highlight
  unsigned char hilightA, hilightB;

  // module details
  unsigned char timeBase, speed1, speed2, arpLen;
  bool pal;
  bool customTempo;
  int hz, patLen, ordersLen, insLen, waveLen, sampleLen;

  DivOrders orders;
  std::vector<DivInstrument*> ins;
  DivChannelData pat[17];
  std::vector<DivWavetable*> wave;
  std::vector<DivSample*> sample;

  DivInstrument nullIns;
  DivWavetable nullWave;

  void unload();

  DivSong():
    version(24),
    system(DIV_SYSTEM_GENESIS),
    name(""),
    author(""),
    carrier(""),
    composer(""),
    vendor(""),
    category(""),
    writer(""),
    arranger(""),
    copyright(""),
    manGroup(""),
    manInfo(""),
    createdDate(""),
    revisionDate(""),
    hilightA(4),
    hilightB(16),
    timeBase(0),
    speed1(6),
    speed2(6),
    arpLen(1),
    pal(false),
    customTempo(false),
    hz(60),
    patLen(64),
    ordersLen(1),
    insLen(0),
    waveLen(0),
    sampleLen(0) {
  }
};
