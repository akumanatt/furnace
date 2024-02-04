/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2023 tildearrow and contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _KURUMITSU_H
#define _KURUMITSU_H

#include "../dispatch.h"
#include "../engine.h"
#include "../waveSynth.h"

class DivPlatformKurumitsu: public DivDispatch {
  struct KrmtEmu {
    unsigned char advCnt;
    unsigned noiseState;
    unsigned char noiseLatch[16];
    int lastData;
    int lastData2;
    KrmtEmu():
      advCnt(0),
      noiseState(0),
      noiseLatch{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      lastData(0),
      lastData2(0) {}
  } emu;

  struct Channel: public SharedChannel<int> {
    int wave, sample;
    unsigned char pan, advNum, advDen;
    unsigned int audPos;
    bool setPos, noise, pcm;
    DivWaveSynth ws;
    void reset() {
        freq=baseFreq=pitch=pitch2=note=0;
        wave=sample=ins=-1;
        pan=255;
        advNum=advDen=0;
        active=false;
        insChanged=freqChanged=true;
        keyOn=keyOff=inPorta=pcm=false;
        vol=outVol=63;
    }
    Channel():
      SharedChannel<int>(63),
      wave(-1),
      sample(-1),
      pan(255),
      advNum(0),
      advDen(0),
      audPos(0),
      setPos(0),
      noise(false),
      pcm(false) {}
  };
  Channel chan[16];
  DivDispatchOscBuffer* oscBuf[16];
  bool isMuted[16];
  unsigned char* sampleMem;
  size_t sampleMemLen;

  unsigned char sampleOff[256];
  unsigned char sampleLoopS[256];
  unsigned char sampleLoopE[256];
  bool sampleLoaded[256];

  unsigned char regPool[0x2000];
  double calcNoteFreq(int ch, int note);
  void updateWave(int ch);
  friend void putDispatchChip(void*,int);
  friend void putDispatchChan(void*,int,int);
  public:
    void acquire(short** buf, size_t len);
    int dispatch(DivCommand c);
    void* getChanState(int chan);
    DivMacroInt* getChanMacroInt(int ch);
    unsigned short getPan(int chan);
    DivChannelPair getPaired(int chan);
    DivChannelModeHints getModeHints(int chan);
    DivSamplePos getSamplePos(int ch);
    DivDispatchOscBuffer* getOscBuffer(int chan);
    unsigned char* getRegisterPool();
    int getRegisterPoolSize();
    void reset();
    void forceIns();
    void tick(bool sysTick=true);
    void muteChannel(int ch, bool mute);
    int getOutputCount();
    float getPostAmp();
    void notifyWaveChange(int wave);
    void notifyInsDeletion(void* ins);
    int mapVelocity(int ch, float vel);
    void poke(unsigned int addr, unsigned short val);
    void poke(std::vector<DivRegWrite>& wlist);
    const void* getSampleMem(int index = 0);
    size_t getSampleMemCapacity(int index = 0);
    size_t getSampleMemUsage(int index = 0);
    bool isSampleLoaded(int index, int sample);
    void renderSamples(int chipID);
    const char** getRegisterSheet();
    int init(DivEngine* parent, int channels, int sugRate, const DivConfig& flags);
    void quit();
    DivPlatformKurumitsu():
      DivDispatch() {}
    ~DivPlatformKurumitsu();
};

#endif
