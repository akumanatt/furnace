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

#include "kurumitsu.h"
#include "../engine.h"
#include "../../ta-log.h"
#include "furIcons.h"
#include <math.h>

#define chRead(c,a) sampleMem[(c)*16+(a)+0x1f00]
#define chWrite(c,a,v) sampleMem[(c)*16+(a)+0x1f00]=(v)

#define CHIP_FREQBASE 4194304

const char* regCheatSheetKurumitsu[]={
  "CHxMod",       "00+x*10",
  "CHxVol",       "01+x*10",
  "CHxPan",       "02+x*10",
  "CHxWaveCur",   "03+x*10",
  "CHxWaveEnd",   "04+x*10",
  "CHxWaveLoop",  "05+x*10",
  "CHxAdvFreqLo", "06+x*10",
  "CHxAdvFreqHi", "07+x*10",
  "CHxAdvAccLo",  "08+x*10",
  "CHxAdvAccHi",  "09+x*10",
  "CHxFreqLo",    "0A+x*10",
  "CHxFreqMe",    "0B+x*10",
  "CHxFreqHi",    "0C+x*10",
  "CHxAccLo",     "0D+x*10",
  "CHxAccMe",     "0E+x*10",
  "CHxAccHi",     "0F+x*10",
  NULL
};

const char** DivPlatformKurumitsu::getRegisterSheet() {
  return regCheatSheetKurumitsu;
}

void DivPlatformKurumitsu::acquire(short** buf, size_t len) {
  const unsigned char volTab[64]={
      0,  1,  1,  1,  1,  1,  1,  1,
      2,  2,  2,  2,  3,  3,  3,  3,
      4,  4,  5,  5,  6,  6,  7,  7,
      8,  9, 10, 11, 12, 13, 14, 15,
     17, 18, 20, 22, 24, 26, 29, 31,
     34, 37, 41, 45, 49, 53, 58, 63,
     69, 75, 82, 90, 98,107,116,127,
    139,151,165,180,196,214,233,255
  };

  for (size_t h=0; h<len; h++) {
    int outAL=0;
    int outAR=0;
    emu.advCnt=(emu.advCnt+1)&0x1f;
    for (int i=0; i<16; i++) {
      int outL=0;
      int outR=0;

      unsigned char mod=chRead(i,0);
      unsigned char vol=chRead(i,1);
      unsigned char pan=chRead(i,2);
      unsigned char wave=chRead(i,3);
      unsigned advfreq=chRead(i,6)|((unsigned)chRead(i,7)<<8);
      unsigned advPhase=chRead(i,8)|((unsigned)chRead(i,9)<<8);
      unsigned freq=chRead(i,10)|((unsigned)chRead(i,11)<<8)|((unsigned)chRead(i,12)<<16);
      unsigned phase=chRead(i,13)|((unsigned)chRead(i,14)<<8)|((unsigned)chRead(i,15)<<16);

      unsigned char mod1=mod&7;
      unsigned char mod2=(mod>>4)&7;
      int vol_modded=volTab[vol&0x3f];
      unsigned phase_modded=phase;
      if (mod1) {
        if (mod&0x08) vol_modded+=emu.lastData>>(7-mod1);
        else phase_modded+=emu.lastData<<mod1;
      }
      if (mod2) {
        if (mod&0x80) vol_modded+=emu.lastData2>>(7-mod2);
        else phase_modded+=emu.lastData2<<mod2;
      }
      vol_modded=CLAMP(vol_modded,0,255);
      unsigned char data=(vol&0x80)?emu.noiseLatch[i]:sampleMem[wave*32+((phase_modded>>16)&31)];

      if (emu.advCnt==0x1f) {
        advPhase+=advfreq;
        if (advPhase&0x10000) {
          if (wave==chRead(i,4)) {
            wave=chRead(i,5);
          } else {
            wave+=1;
          }
        }
      }
      unsigned phase_old=phase;
      phase+=freq&0xfffff;
      if ((vol&0x80)&&((phase^phase_old)&0x10000)) {
        data=emu.noiseState&0xff;
        emu.noiseLatch[i]=data;
      }
      emu.noiseState=(emu.noiseState>>1)|((~(emu.noiseState^(emu.noiseState>>1))&1)<<14);

      chWrite(i,3,wave);
      chWrite(i,8,advPhase&0xff);
      chWrite(i,9,(advPhase>>8)&0xff);
      chWrite(i,13,phase&0xff);
      chWrite(i,14,(phase>>8)&0xff);
      chWrite(i,15,(phase>>16)&0x1f);

      int out=(signed char)data;
      out*=vol_modded;
      emu.lastData2=emu.lastData;
      emu.lastData=out;
      oscBuf[i]->data[oscBuf[i]->needle++]=isMuted[i]?0:out;
      outAL+=(out*(pan&0xf))>>8;
      outAR+=(out*((pan>>4)&0xf))>>8;
    }
    buf[0][h]=outAL;
    buf[1][h]=outAR;
  }
}

double DivPlatformKurumitsu::calcNoteFreq(int ch, int note) {
  if (chan[ch].pcm) { // PCM note
    double off=4194304.0;
    int sample=chan[ch].sample;
    if (sample>=0 && sample<parent->song.sampleLen) {
      DivSample* s=parent->getSample(sample);
      if (s->centerRate<1) {
        off=4194304.0;
      } else {
        off=4194304.0*(s->centerRate/8363.0);
      }
    }
    return parent->calcBaseFreq(chipClock,off,note,false);
  }
  // Wavetable note
  return NOTE_FREQUENCY(note);
}

void DivPlatformKurumitsu::updateWave(int ch) {
  for (int i=0; i<32; i++) {
    int data=chan[ch].ws.output[i]-128;
    sampleMem[ch*32+i]=(unsigned char)data;
  }
}

void DivPlatformKurumitsu::tick(bool sysTick) {
  for (int i=0; i<16; i++) {
    chan[i].std.next();
    if (chan[i].std.vol.had) {
      chan[i].outVol=MAX((chan[i].vol&63)+chan[i].std.vol.val-63,0);
      chWrite(i,1,(chRead(i,1)&~0x3f)|((unsigned char)chan[i].outVol));
    }
    if (chan[i].std.duty.had) {
      chan[i].noise=chan[i].std.duty.val;
      chWrite(i,1,(chRead(i,1)&0x7f)|(unsigned char)(chan[i].std.duty.val<<7));
    }
    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had) {
      if (!chan[i].inPorta) {
        chan[i].baseFreq=calcNoteFreq(i,parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].std.wave.had && !chan[i].pcm) {
      if (chan[i].wave!=chan[i].std.wave.val || chan[i].ws.activeChanged()) {
        chan[i].wave=chan[i].std.wave.val;
        if (!chan[i].pcm) {
          chan[i].ws.changeWave1(chan[i].wave);
        }
      }
    }
    if (chan[i].std.panL.had) {
      chan[i].pan&=0xf0;
      chan[i].pan|=chan[i].std.panL.val&15;
      chWrite(i,2,isMuted[i]?0:chan[i].pan);
    }
    if (chan[i].std.panR.had) {
      chan[i].pan&=0x0f;
      chan[i].pan|=(chan[i].std.panR.val&15)<<4;
      chWrite(i,2,isMuted[i]?0:chan[i].pan);
    }
    if (chan[i].std.pitch.had) {
      if (chan[i].std.pitch.mode) {
        chan[i].pitch2+=chan[i].std.pitch.val;
        CLAMP_VAR(chan[i].pitch2,-65535,65535);
      } else {
        chan[i].pitch2=chan[i].std.pitch.val;
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].std.phaseReset.had) {
      if (chan[i].std.phaseReset.val && chan[i].active) {
        if (chan[i].pcm) {
          chWrite(i,8,0);
          chWrite(i,9,0);
          emu.advCnt=0;
        }
        chWrite(i,13,0);
        chWrite(i,14,0);
        chWrite(i,15,chRead(i,15)&~0xf);
      }
    }
    if (chan[i].active) {
      if (chan[i].ws.tick()) {
        updateWave(i);
      }
    }
    if (chan[i].setPos) {
      // force keyon
      chan[i].keyOn=true;
      chan[i].setPos=false;
    } else {
      chan[i].audPos=0;
    }
    if (chan[i].freqChanged || chan[i].keyOn) {
      double off=4194304.0;
      if (chan[i].sample>=0 && chan[i].sample<parent->song.sampleLen) {
        DivSample* s=parent->getSample(chan[i].sample);
        if (s->centerRate<1) {
          off=4194304.0;
        } else {
          off=4194304.0*(s->centerRate/8363.0);
        }
      }
      chan[i].freq=parent->calcFreq(chan[i].baseFreq,chan[i].pitch,chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,chan[i].fixedArp,false,2,chan[i].pitch2,chipClock,chan[i].pcm?off:CHIP_FREQBASE);
      if (chan[i].freq>1048575) chan[i].freq=1048575;
      if (chan[i].pcm) {
        int advNum=MAX(chan[i].advNum,1);
        int advDen=MAX(chan[i].advDen,1);
        int advFreq=chan[i].freq*advNum/advDen;
        while (advFreq>65535) {
          advFreq/=2;
        }
        chWrite(i,6,advFreq&0xff);
        chWrite(i,7,(advFreq>>8)&0xff);
        if (chan[i].keyOn){
          if (chan[i].audPos>0) {
            chWrite(i,3,sampleOff[chan[i].sample]+chan[i].audPos/32);
            chWrite(i,8,0);
            chWrite(i,9,0);
            chWrite(i,13,0);
            chWrite(i,14,0);
            chWrite(i,15,0);
          }
          chan[i].keyOn=false;
        }
      }
      chWrite(i,10,chan[i].freq&0xff);
      chWrite(i,11,(chan[i].freq>>8)&0xff);
      chWrite(i,12,((chan[i].freq>>16)&0xf)|(chRead(i,12)&~0xf));
      chan[i].freqChanged=false;
    }
  }
}

int DivPlatformKurumitsu::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_KURUMITSU);
      chan[c.chan].pcm=ins->amiga.useSample;
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].sample=ins->amiga.getSample(c.value);
        chan[c.chan].baseFreq=calcNoteFreq(c.chan, c.value);
        chan[c.chan].freqChanged=true;
        chan[c.chan].note=c.value;
      }
      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].macroInit(ins);
      if (!parent->song.brokenOutVol && !chan[c.chan].std.vol.will) {
        chan[c.chan].outVol=chan[c.chan].vol;
      }
      if (chan[c.chan].pcm) {
        if (chan[c.chan].sample<0 || chan[c.chan].sample>=parent->song.sampleLen) {
          chan[c.chan].sample=-1;
        } else {
          chWrite(c.chan,3,sampleOff[chan[c.chan].sample]);
          chWrite(c.chan,4,sampleLoopE[chan[c.chan].sample]);
          chWrite(c.chan,5,sampleLoopS[chan[c.chan].sample]);
        }
        chan[c.chan].wave=-1;
        chWrite(c.chan,8,0);
        chWrite(c.chan,9,0);
        chWrite(c.chan,13,0);
        chWrite(c.chan,14,0);
        chWrite(c.chan,15,0);
        emu.advCnt=0;
      } else {
        if (chan[c.chan].wave<0) {
          chan[c.chan].wave=0;
          chan[c.chan].ws.changeWave1(chan[c.chan].wave);
        }
        chan[c.chan].ws.init(ins,32,255,chan[c.chan].insChanged);
        chWrite(c.chan,3,c.chan);
        chWrite(c.chan,6,0);
        chWrite(c.chan,7,0);
      }
      chWrite(c.chan,1,(chRead(c.chan,1)&~0x3f)|((unsigned char)chan[c.chan].vol));
      chan[c.chan].insChanged=false;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].pcm=false;
      chan[c.chan].active=false;
      chan[c.chan].macroInit(NULL);
      chWrite(c.chan,1,0);
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
      chan[c.chan].vol=c.value&0x3f;
      if (chan[c.chan].active) {
        chWrite(c.chan,1,(chRead(c.chan,1)&~0x3f)|((unsigned char)chan[c.chan].vol));
      }
      break;
    case DIV_CMD_GET_VOLUME:
      return chan[c.chan].vol;
      break;
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_WAVE:
      chan[c.chan].wave=c.value;
      chan[c.chan].ws.changeWave1(chan[c.chan].wave);
      break;
    case DIV_CMD_NOTE_PORTA: {
      int destFreq=calcNoteFreq(c.chan,c.value2);
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
    case DIV_CMD_PANNING: {
      chan[c.chan].pan=(c.value2&0xf0)|(c.value>>4);
      chWrite(c.chan,2,isMuted[c.chan]?0:chan[c.chan].pan);
      break;
    }
    case DIV_CMD_LEGATO:
      chan[c.chan].note=c.value;
      chan[c.chan].baseFreq=calcNoteFreq(c.chan,chan[c.chan].note+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_KURUMITSU));
      }
      if (!chan[c.chan].inPorta && c.value && !parent->song.brokenPortaArp && chan[c.chan].std.arp.will && !NEW_ARP_STRAT) chan[c.chan].baseFreq=calcNoteFreq(c.chan,chan[c.chan].note);
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_STD_NOISE_MODE:
      chan[c.chan].noise=c.value;
      chWrite(c.chan,1,(chRead(c.chan,1)&0x7f)|(unsigned char)(c.value<<7));
      break;
    case DIV_CMD_SAMPLE_POS:
      chan[c.chan].audPos=c.value;
      chan[c.chan].setPos=true;
      break;
    case DIV_CMD_KURUMITSU_MOD:
      chWrite(c.chan,0,c.value);
      break;
    case DIV_CMD_KURUMITSU_ADV_LOW:
      if (chan[c.chan].pcm) {
        chWrite(c.chan,6,(unsigned char)c.value);
      }
      break;
    case DIV_CMD_KURUMITSU_ADV_HIGH:
      if (chan[c.chan].pcm) {
        chWrite(c.chan,7,(unsigned char)c.value);
      }
      break;
    case DIV_CMD_KURUMITSU_ADV_RATIO:
      chan[c.chan].advDen=c.value&0xf;
      chan[c.chan].advNum=(c.value>>4)&0xf;
      break;
    case DIV_CMD_GET_VOLMAX:
      return 63;
      break;
    case DIV_CMD_MACRO_OFF:
      chan[c.chan].std.mask(c.value,true);
      break;
    case DIV_CMD_MACRO_ON:
      chan[c.chan].std.mask(c.value,false);
      break;
    default:
      break;
  }
  return 1;
}

void DivPlatformKurumitsu::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
  chWrite(ch,2,mute?0:chan[ch].pan);
}

void DivPlatformKurumitsu::forceIns() {
  for (int i=0; i<16; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;
    updateWave(i);
  }
}

void* DivPlatformKurumitsu::getChanState(int ch) {
  return &chan[ch];
}

DivMacroInt* DivPlatformKurumitsu::getChanMacroInt(int ch) {
  return &chan[ch].std;
}

unsigned short DivPlatformKurumitsu::getPan(int ch) {
  unsigned short pan=chRead(ch,2);
  return ((pan&15)<<8)|((pan&0xf0)>>4);
}

DivChannelPair DivPlatformKurumitsu::getPaired(int ch) {
  unsigned char mod=chRead(ch,0);
  signed char p1=(mod&0x07)?(ch-1)&15:-1;
  signed char p2=(mod&0x70)?(ch-2)&15:-1;
  return DivChannelPair("mod", p1, p2);
}

DivChannelModeHints DivPlatformKurumitsu::getModeHints(int ch) {
  DivChannelModeHints ret;
  ret.count=1;
  ret.hint[0]=ICON_FUR_NOISE;
  ret.type[0]=0;

  if (chan[ch].noise) ret.type[0]=4;
  
  return ret;
}

DivSamplePos DivPlatformKurumitsu::getSamplePos(int ch) {
  if (!chan[ch].pcm) return DivSamplePos();
  int sample=chan[ch].sample;
  int pos=(int)chRead(ch,3)*32+(chRead(ch,15)&0x1f)-sampleOff[sample];
  int rate=chRead(ch,10)|((int)chRead(ch,11)<<8)|((int)chRead(ch,12)<<16);
  rate=(long long)rate*chipClock/4194304;
  return DivSamplePos(sample,pos,rate);
}

DivDispatchOscBuffer* DivPlatformKurumitsu::getOscBuffer(int ch) {
  return oscBuf[ch];
}

unsigned char* DivPlatformKurumitsu::getRegisterPool() {
  return &sampleMem[0x1f00];
}

int DivPlatformKurumitsu::getRegisterPoolSize() {
  return 256;
}

void DivPlatformKurumitsu::reset() {
  emu=KrmtEmu();
  memset(&sampleMem[0x1f00],0,256);
  for (int i=0; i<16; i++) {
    chan[i]=DivPlatformKurumitsu::Channel();
    chan[i].reset();
    chan[i].std.setEngine(parent);
    chan[i].ws.setEngine(parent);
    chan[i].ws.init(NULL,32,255,false);
    chWrite(i,2,isMuted[i]?0:255);
  }
}

int DivPlatformKurumitsu::getOutputCount() {
  return 2;
}

float DivPlatformKurumitsu::getPostAmp() {
  return 4.0f;
}

void DivPlatformKurumitsu::notifyWaveChange(int wave) {
  for (int i=0; i<16; i++) {
    if (chan[i].wave==wave) {
      chan[i].ws.changeWave1(wave);
      updateWave(i);
    }
  }
}

void DivPlatformKurumitsu::notifyInsDeletion(void* ins) {
  for (int i=0; i<16; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

int DivPlatformKurumitsu::mapVelocity(int ch, float vel) {
  const int volMax=MAX(1,dispatch(DivCommand(DIV_CMD_GET_VOLMAX,MAX(ch,0))));
  double attenDb=20*log10(vel); // 20dB/decade for a linear mapping
  double attenUnits=attenDb/0.75; // 0.75dB/unit
  return MAX(0,volMax+attenUnits);
}

void DivPlatformKurumitsu::poke(unsigned int addr, unsigned short val) {
  sampleMem[addr]=val;
}

void DivPlatformKurumitsu::poke(std::vector<DivRegWrite>& wlist) {
  for (DivRegWrite& i: wlist) sampleMem[i.addr]=i.val;
}

const void* DivPlatformKurumitsu::getSampleMem(int index) {
  return index >= 0 ? sampleMem : 0;
}

size_t DivPlatformKurumitsu::getSampleMemCapacity(int index) {
  return index == 0 ? 0x2000 : 0;
}

size_t DivPlatformKurumitsu::getSampleMemUsage(int index) {
  return index >= 0 ? (sampleMemLen+256) : 0;
}

bool DivPlatformKurumitsu::isSampleLoaded(int index, int sample) {
  if (index!=0) return false;
  if (sample<0 || sample>255) return false;
  return sampleLoaded[sample];
}

void DivPlatformKurumitsu::renderSamples(int sysID) {
  memset(sampleMem,0,0x1f00);
  memset(sampleOff,0,256*sizeof(unsigned char));
  memset(sampleLoopS,0,256*sizeof(unsigned char));
  memset(sampleLoopE,0,256*sizeof(unsigned char));
  memset(sampleLoaded,0,256*sizeof(bool));

  size_t memPos=16*32;
  for (int i=0; i<parent->song.sampleLen; i++) {
    DivSample* s=parent->song.sample[i];
    if (!s->renderOn[0][sysID]) {
      continue;
    }
    if (memPos>=0x1f00) {
      logW("out of Kurumitsu memory for sample %d!",i);
      break;
    }
    int paddedLen=(s->length8+32)&~31;
    if (memPos+paddedLen>=0x1f00) {
      memcpy(sampleMem+memPos,s->data8,0x1f00-memPos);
      logW("out of Kurumitsu memory for sample %d!",i);
    } else {
      memcpy(sampleMem+memPos,s->data8,paddedLen);
      sampleLoaded[i]=true;
    }
    if (!s->loop) {
      // add one more silent wave slot so one-shot works
      paddedLen+=32;
      sampleLoopS[i]=sampleLoopE[i]=(unsigned char)MIN((memPos+paddedLen-1)/32,247);
    } else {
      sampleLoopS[i]=(unsigned char)MIN((memPos+s->loopStart)/32,247);
      sampleLoopE[i]=(unsigned char)MIN((memPos+s->loopEnd-1)/32,247);
    }
    sampleOff[i]=(unsigned char)(memPos/32);
    memPos+=paddedLen;
  }
  sampleMemLen=memPos;
}

int DivPlatformKurumitsu::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  for (int i=0; i<16; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }
  sampleMem=new unsigned char[0x2000];
  sampleMemLen=16*32;
  chipClock=4000000;
  CHECK_CUSTOM_CLOCK;
  rate=chipClock/64;
  for (int i=0; i<16; i++) {
    oscBuf[i]->rate=rate;
  }
  reset();
  return 16;
}

void DivPlatformKurumitsu::quit() {
  for (int i=0; i<16; i++) {
    delete oscBuf[i];
  }
  delete[] sampleMem;
}

DivPlatformKurumitsu::~DivPlatformKurumitsu() {
}
