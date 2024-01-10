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

#define _USE_MATH_DEFINES
#include "gbadma.h"
#include "../engine.h"
#include "../filter.h"
#include <math.h>

#define CHIP_DIVIDER 16

void DivPlatformGBADMA::acquire(short** buf, size_t len) {
  // HLE for now
  int outL[2]={0,0};
  int outR[2]={0,0};
  for (size_t h=0; h<len; h++) {
    // internal mixing is always 10-bit
    for (int i=0; i<2; i++) {
      bool newSamp=h==0;
      if (chan[i].active && (chan[i].useWave || (chan[i].sample>=0 && chan[i].sample<parent->song.sampleLen))) {
        chan[i].audSub+=(1<<outDepth);
        if (chan[i].useWave) {
          if (chan[i].audPos<(int)chan[i].audLen) {
            chan[i].audDat=chan[i].ws.output[chan[i].audPos]-0x80;
          } else {
            chan[i].audDat=0;
          }
          newSamp=true;
          if (chan[i].audSub>=chan[i].freq) {
            int posInc=chan[i].audSub/chan[i].freq;
            chan[i].audSub-=chan[i].freq*posInc;
            chan[i].audPos+=posInc;
            chan[i].dmaCount+=posInc;
            if (chan[i].dmaCount>=16 && chan[i].audPos>=(int)chan[i].audLen) {
              chan[i].audPos%=chan[i].audLen;
            }
            chan[i].dmaCount&=15;
          }
        } else {
          DivSample* s=parent->getSample(chan[i].sample);
          if (s->samples>0) {
            if (chan[i].audPos>=0 && chan[i].audPos<(int)s->samples) {
              chan[i].audDat=s->data8[chan[i].audPos];
            } else {
              chan[i].audDat=0;
            }
            newSamp=true;
            if (chan[i].audSub>=chan[i].freq) {
              int posInc=chan[i].audSub/chan[i].freq;
              chan[i].audSub-=chan[i].freq*posInc;
              chan[i].audPos+=posInc;
              chan[i].dmaCount+=posInc;
              if (s->isLoopable()) {
                if (chan[i].dmaCount>=16 && chan[i].audPos>=s->loopEnd) {
                  int loopPos=chan[i].audPos-s->loopStart;
                  chan[i].audPos=(loopPos%(s->loopEnd-s->loopStart))+s->loopStart;
                }
              } else if (chan[i].audPos>=(int)s->samples) {
                chan[i].sample=-1;
              }
              chan[i].dmaCount&=15;
            }
          } else {
            chan[i].sample=-1;
            chan[i].audSub=0;
            chan[i].audPos=0;
          }
        }
      } else {
        chan[i].audDat=0;
      }
      if (!isMuted[i] && newSamp) {
        int out=chan[i].audDat*(chan[i].vol*chan[i].envVol/2)<<1;
        outL[i]=(chan[i].pan&2)?out:0;
        outR[i]=(chan[i].pan&1)?out:0;
      }
      oscBuf[i]->data[oscBuf[i]->needle++]=(short)((outL[i]+outR[i])<<5);
    }
    int l=outL[0]+outL[1];
    int r=outR[0]+outR[1];
    l=(l>>(10-outDepth))<<(16-outDepth);
    r=(r>>(10-outDepth))<<(16-outDepth);
    if (l<-32768) l=-32768;
    if (l>32767) l=32767;
    if (r<-32768) r=-32768;
    if (r>32767) r=32767;
    buf[0][h]=(short)l;
    buf[1][h]=(short)r;
  }
}

void DivPlatformGBADMA::tick(bool sysTick) {
  for (int i=0; i<2; i++) {
    DivInstrument* ins=parent->getIns(chan[i].ins,DIV_INS_AMIGA);
    chan[i].std.next();
    if (chan[i].std.vol.had) {
      chan[i].envVol=chan[i].std.vol.val;
      if (ins->type==DIV_INS_AMIGA) chan[i].envVol/=32;
    }
    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had) {
      if (!chan[i].inPorta) {
        chan[i].baseFreq=NOTE_PERIODIC(parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].useWave && chan[i].std.wave.had) {
      if (chan[i].wave!=chan[i].std.wave.val || chan[i].ws.activeChanged()) {
        chan[i].wave=chan[i].std.wave.val;
        chan[i].ws.changeWave1(chan[i].wave);
        if (!chan[i].keyOff) chan[i].keyOn=true;
      }
    }
    if (chan[i].useWave && chan[i].active) {
      chan[i].ws.tick();
    }
    if (chan[i].std.pitch.had) {
      if (chan[i].std.pitch.mode) {
        chan[i].pitch2+=chan[i].std.pitch.val;
        CLAMP_VAR(chan[i].pitch2,-32768,32767);
      } else {
        chan[i].pitch2=chan[i].std.pitch.val;
      }
      chan[i].freqChanged=true;
    }
    if (ins->type==DIV_INS_AMIGA) {
      if (chan[0].std.panL.had) {
        chan[0].pan=(chan[0].pan&~2)|(chan[0].std.panL.val>0?2:0);
      }
      if (chan[0].std.panR.had) {
        chan[0].pan=(chan[0].pan&~1)|(chan[0].std.panR.val>0?1:0);
      }
    } else {
      if (chan[i].std.panL.had) {
        chan[i].pan=chan[i].std.panL.val&3;
      }
    }
    if (chan[i].std.phaseReset.had && chan[i].std.phaseReset.val==1) {
      chan[i].audPos=0;
    }
    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
      double off=1.0;
      if (!chan[i].useWave && chan[i].sample>=0 && chan[i].sample<parent->song.sampleLen) {
        DivSample* s=parent->getSample(chan[i].sample);
        off=(s->centerRate>=1)?(8363.0/(double)s->centerRate):1.0;
      }
      chan[i].freq=off*parent->calcFreq(chan[i].baseFreq,chan[i].pitch,chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,chan[i].fixedArp,true,0,chan[i].pitch2,chipClock,CHIP_DIVIDER);

      // emulate prescaler rounding
      if (chan[i].freq<65536) {
        if (chan[i].freq<1) chan[i].freq=1;
      } else if (chan[i].freq<65536*64) {
        chan[i].freq=chan[i].freq&~63;
      } else if (chan[i].freq<65536*256) {
        chan[i].freq=chan[i].freq&~255;
      } else {
        chan[i].freq=chan[i].freq&~1024;
        if (chan[i].freq>65536*1024) chan[i].freq=65536*1024;
      }
      if (chan[i].keyOn) {
        if (!chan[i].std.vol.had) {
          chan[i].envVol=2;
        }
        chan[i].keyOn=false;
      }
      if (chan[i].keyOff) {
        chan[i].keyOff=false;
      }
      chan[i].freqChanged=false;
    }
  }
}

int DivPlatformGBADMA::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_AMIGA);
      if (ins->amiga.useWave) {
        chan[c.chan].useWave=true;
        chan[c.chan].audLen=ins->amiga.waveLen+1;
        if (chan[c.chan].insChanged) {
          if (chan[c.chan].wave<0) {
            chan[c.chan].wave=0;
            chan[c.chan].ws.setWidth(chan[c.chan].audLen);
            chan[c.chan].ws.changeWave1(chan[c.chan].wave);
          }
        }
      } else {
        if (c.value!=DIV_NOTE_NULL) {
          chan[c.chan].sample=ins->amiga.getSample(c.value);
          c.value=ins->amiga.getFreq(c.value);
        }
        chan[c.chan].useWave=false;
      }
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].baseFreq=NOTE_PERIODIC(c.value);
      }
      if (chan[c.chan].useWave || chan[c.chan].sample<0 || chan[c.chan].sample>=parent->song.sampleLen) {
        chan[c.chan].sample=-1;
      }
      if (chan[c.chan].setPos) {
        chan[c.chan].setPos=false;
      } else {
        chan[c.chan].audPos=0;
      }
      chan[c.chan].audSub=0;
      chan[c.chan].audDat=0;
      chan[c.chan].dmaCount=0;
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].freqChanged=true;
        chan[c.chan].note=c.value;
      }
      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].macroInit(ins);
      if (!parent->song.brokenOutVol && !chan[c.chan].std.vol.will) {
        chan[c.chan].envVol=2;
      }
      if (chan[c.chan].useWave) {
        chan[c.chan].ws.init(ins,chan[c.chan].audLen,255,chan[c.chan].insChanged);
      }
      chan[c.chan].insChanged=false;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].sample=-1;
      chan[c.chan].active=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].macroInit(NULL);
      break;
    case DIV_CMD_NOTE_OFF_ENV:
    case DIV_CMD_ENV_RELEASE:
      chan[c.chan].std.release();
      break;
    case DIV_CMD_INSTRUMENT:
      if (chan[c.chan].ins!=c.value || c.value2==1) {
        chan[c.chan].ins=c.value;
        chan[c.chan].insChanged=true;
      }
      break;
    case DIV_CMD_VOLUME:
      if (chan[c.chan].vol!=c.value) {
        chan[c.chan].vol=c.value;
        if (!chan[c.chan].std.vol.has) {
          chan[c.chan].envVol=2;
        }
      }
      break;
    case DIV_CMD_GET_VOLUME:
      return chan[c.chan].vol;
      break;
    case DIV_CMD_PANNING:
      chan[c.chan].pan=0;
      chan[c.chan].pan|=(c.value>0)?2:0;
      chan[c.chan].pan|=(c.value2>0)?1:0;
      break;
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_WAVE:
      if (!chan[c.chan].useWave) break;
      chan[c.chan].wave=c.value;
      chan[c.chan].keyOn=true;
      chan[c.chan].ws.changeWave1(chan[c.chan].wave);
      break;
    case DIV_CMD_NOTE_PORTA: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_AMIGA);
      chan[c.chan].sample=ins->amiga.getSample(c.value2);
      int destFreq=NOTE_PERIODIC(c.value2);
      bool return2=false;
      if (destFreq>chan[c.chan].baseFreq) {
        chan[c.chan].baseFreq+=c.value;
        if (chan[c.chan].baseFreq>=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      } else {
        chan[c.chan].baseFreq-=c.value;
        if (chan[c.chan].baseFreq<=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      }
      chan[c.chan].freqChanged=true;
      if (return2) {
        chan[c.chan].inPorta=false;
        return 2;
      }
      break;
    }
    case DIV_CMD_LEGATO: {
      chan[c.chan].baseFreq=NOTE_PERIODIC(c.value+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].freqChanged=true;
      chan[c.chan].note=c.value;
      break;
    }
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_AMIGA));
      }
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_SAMPLE_POS:
      if (chan[c.chan].useWave) break;
      chan[c.chan].audPos=c.value;
      chan[c.chan].setPos=true;
      break;
    case DIV_CMD_GET_VOLMAX:
      return 2;
      break;
    case DIV_CMD_MACRO_OFF:
      chan[c.chan].std.mask(c.value,true);
      break;
    case DIV_CMD_MACRO_ON:
      chan[c.chan].std.mask(c.value,false);
      break;
    case DIV_ALWAYS_SET_VOLUME:
      return 1;
      break;
    default:
      break;
  }
  return 1;
}

void DivPlatformGBADMA::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
}

void DivPlatformGBADMA::forceIns() {
  for (int i=0; i<2; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;
    chan[i].audPos=0;
    chan[i].sample=-1;
  }
}

void* DivPlatformGBADMA::getChanState(int ch) {
  return &chan;
}

DivDispatchOscBuffer* DivPlatformGBADMA::getOscBuffer(int ch) {
  return oscBuf[ch];
}

void DivPlatformGBADMA::reset() {
  for (int i=0; i<2; i++) {
    chan[i]=DivPlatformGBADMA::Channel();
    chan[i].std.setEngine(parent);
    chan[i].ws.setEngine(parent);
    chan[i].ws.init(NULL,32,255);
    chan[i].audDat=0;
  }
}

int DivPlatformGBADMA::getOutputCount() {
  return 2;
}

DivMacroInt* DivPlatformGBADMA::getChanMacroInt(int ch) {
  return &chan[ch].std;
}

unsigned short DivPlatformGBADMA::getPan(int ch) {
  return ((chan[ch].pan&2)<<7)|(chan[ch].pan&1);
}

DivSamplePos DivPlatformGBADMA::getSamplePos(int ch) {
  if (ch>=2) return DivSamplePos();
  return DivSamplePos(
    chan[ch].sample,
    chan[ch].audPos,
    chan[ch].freq
  );
}

void DivPlatformGBADMA::notifyInsChange(int ins) {
  for (int i=0; i<2; i++) {
    if (chan[i].ins==ins) {
      chan[i].insChanged=true;
    }
  }
}

void DivPlatformGBADMA::notifyWaveChange(int wave) {
  for (int i=0; i<2; i++) {
    if (chan[i].useWave && chan[i].wave==wave) {
      chan[i].ws.changeWave1(wave);
    }
  }
}

void DivPlatformGBADMA::notifyInsDeletion(void* ins) {
  for (int i=0; i<2; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

void DivPlatformGBADMA::setFlags(const DivConfig& flags) {
  outDepth=flags.getInt("dacDepth",9);
  chipClock=1<<24;
  CHECK_CUSTOM_CLOCK;
  rate=chipClock>>outDepth;
  for (int i=0; i<2; i++) {
    oscBuf[i]->rate=rate;
  }
}

int DivPlatformGBADMA::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  dumpWrites=false;
  skipRegisterWrites=false;
  for (int i=0; i<2; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }
  setFlags(flags);
  reset();
  return 2;
}

void DivPlatformGBADMA::quit() {
  for (int i=0; i<2; i++) {
    delete oscBuf[i];
  }
}
