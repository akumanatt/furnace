/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2024 tildearrow and contributors
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

#include "xgm.h"
#include "../engine.h"
#include "../ta-log.h"
#include <fmt/printf.h>

constexpr int MASTER_CLOCK_PREC=(sizeof(void*)==8)?8:0;
constexpr int MASTER_CLOCK_MASK=(sizeof(void*)==8)?0xff:0;
// optimize order for channel and freq latch
static const int REG_ORDER[] = {
  0x010,0x011,0x012,0x014,0x015,0x016,0x018,0x019,0x01a,0x01c,0x01d,0x01e,      // PSG
  0x022,0x024,0x025,0x027,0x02a,                                                // FM global
  0x030,0x050,0x060,0x070,0x080,0x090,0x034,0x054,0x064,0x074,0x084,0x094,      // FM 1
  0x038,0x058,0x068,0x078,0x088,0x098,0x03c,0x05c,0x06c,0x07c,0x08c,0x09c,0x0b0,
  0x031,0x051,0x061,0x071,0x081,0x091,0x035,0x055,0x065,0x075,0x085,0x095,      // FM 2
  0x039,0x059,0x069,0x079,0x089,0x099,0x03d,0x05d,0x06d,0x07d,0x08d,0x09d,0x0b1,
  0x032,0x052,0x062,0x072,0x082,0x092,0x036,0x056,0x066,0x076,0x086,0x096,      // FM 3
  0x03a,0x05a,0x06a,0x07a,0x08a,0x09a,0x03e,0x05e,0x06e,0x07e,0x08e,0x09e,0x0b2,
  0x130,0x150,0x160,0x170,0x180,0x190,0x134,0x154,0x164,0x174,0x184,0x194,      // FM 4
  0x138,0x158,0x168,0x178,0x188,0x198,0x13c,0x15c,0x16c,0x17c,0x18c,0x19c,0x1b0,
  0x131,0x151,0x161,0x171,0x181,0x191,0x135,0x155,0x165,0x175,0x185,0x195,      // FM 5
  0x139,0x159,0x169,0x179,0x189,0x199,0x13d,0x15d,0x16d,0x17d,0x18d,0x19d,0x1b1,
  0x132,0x152,0x162,0x172,0x182,0x192,0x136,0x156,0x166,0x176,0x186,0x196,      // FM 6
  0x13a,0x15a,0x16a,0x17a,0x18a,0x19a,0x13e,0x15e,0x16e,0x17e,0x18e,0x19e,0x1b2,
  0x0b4,0x0b5,0x0b6,0x1b4,0x1b5,0x1b6,                                          // FM pan/PMS/AMS
  0x040,0x044,0x048,0x04c,0x041,0x045,0x049,0x04d,0x042,0x046,0x04a,0x04e,      // FM TL
  0x140,0x144,0x148,0x14c,0x141,0x145,0x149,0x14d,0x142,0x146,0x14a,0x14e,
  0x0a4,0x0a0,0x0a5,0x0a1,0x0a6,0x0a2,0x0ac,0x0a8,0x0ad,0x0a9,0x0ae,0x0aa,      // FM freq
  0x1a4,0x1a0,0x1a5,0x1a1,0x1a6,0x1a2
};
const int FM_FREQLS[] = {0xa0,0xa1,0xa2,0xa8,0xa9,0xaa,0x1a0,0x1a1,0x1a2};

class XGMBackRef {
  public:
    int pos;
    inline void add(unsigned int addr, unsigned int val) {
      data[pos]={addr,val};
      pos=(pos+1)&0xff;
    };
    inline DivRegWrite& operator[](int addr) {
      return data[addr];
    }
    inline DivRegWrite& rel(int addr) {
      return data[(addr+pos)&0xff];
    }
    XGMBackRef(): pos(0) {
      for (int i=0; i<256; i++) data[i]=DivRegWrite();
    };
  private:
    DivRegWrite data[256];
};

struct XGMChangedReg {
  short addr;
  unsigned char val;
  unsigned char match;
  XGMChangedReg(int a, unsigned char v, unsigned char m):
    addr(a),
    val(v),
    match(m) {}
  XGMChangedReg():
    addr(-1),
    val(0),
    match(0) {}
};

using XGMBackRefMatch=std::pair<int,int>;

// cleaned up from AArt1256's SSDPCM2 C encoder, modified to skip on 32K boundaries
static std::vector<unsigned char> encode_ssdpcm2(signed char* data, size_t len, int bankOffset) {
  const int deltaMuls[4]={-1,0,1,2};
  const int BLOCK_SIZE=128;
  const int MAX_DELTA=127;
  std::vector<unsigned char> outData;
  int outVal=0;
  for (size_t bpos=0; bpos<len; bpos+=BLOCK_SIZE) {
    signed char buf[BLOCK_SIZE];
    int i=MIN(len-bpos,BLOCK_SIZE);
    memcpy(buf,data+bpos,i);
    while (i<BLOCK_SIZE) {
      buf[i++]=0;
    }
    // test compression on all deltas
    signed char minCodes[BLOCK_SIZE];
    int minErr=INT_MAX;
    int minDelta=0;
    int nextVal=0;
    for (int delta=0; delta<=MAX_DELTA; delta++) {
      signed char testCodes[BLOCK_SIZE];
      int testVal=outVal;
      int testErr=0;
      for (int i=0; i<BLOCK_SIZE; i++) {
        int minCode=0;
        int minNextVal=0;
        int minErr=INT_MAX;
        for (int j=0; j<4; j++) {
          int nextVal=testVal+(delta*deltaMuls[j]);
          // disallow this code from testing if it overflows
          if (nextVal<-128 || nextVal>=128) continue;
          int err=abs((int)buf[i]-nextVal);
          if (err<minErr) {
            minCode=j;
            minErr=err;
            minNextVal=nextVal;
          }
        }
        testCodes[i]=minCode;
        testVal=minNextVal;
        testErr+=minErr;
      }
      if (testErr<minErr) {
        minErr=testErr;
        minDelta=delta;
        nextVal=testVal;
        memcpy(minCodes,testCodes,sizeof(minCodes));
      }
    }
    outVal=nextVal;
    // if it can't fit the current 32K bank, write bank end and align
    bankOffset&=0x7fff;
    if (bankOffset>=(0x8000-BLOCK_SIZE/4-2)) {
      outData.push_back(0xfe); // bank end
      while (++bankOffset!=0x8000) outData.push_back(0); // padding
    }
    outData.push_back(minDelta);
    unsigned char byte=0;
    unsigned char bitCount=0;
    for (int i=0; i<BLOCK_SIZE; i++) {
      byte=(byte<<2)|minCodes[i];
      bitCount++;
      if (bitCount==4) {
        outData.push_back(byte);
        byte=0;
        bitCount=0;
      }
    }
    bankOffset+=BLOCK_SIZE/4+1;
  }
  outData.push_back(0xff); // sample end
  return outData;
}

static void writeCommand(unsigned char number, SafeWriter* w, const std::vector<unsigned char>& values, size_t start, size_t len, bool singleAddr, unsigned int addrOffset, XGMBackRef& backref) {
  if (values.empty() || len==0) return;
  unsigned int valueLen=singleAddr?1:2;
  start*=valueLen;
  len*=valueLen;
  const unsigned char* data=values.data()+start;
  if (singleAddr) {
    for (size_t i=0; i<len; i++) {
      backref.add(addrOffset,data[i]);
    }
  } else {
    for (size_t i=0; i<len; i+=2) {
      // don't add ch3 mode writes into backref since it's ignored by the driver
      if (data[i]!=0x27) backref.add(addrOffset+data[i],data[i+1]);
    }
  }
  while (len>0) {
    size_t thisLen=MIN(len,16*valueLen);
    w->writeC(number|(thisLen/valueLen-1));
    w->write(data,thisLen);
    data+=thisLen;
    len-=thisLen;
  }
}

static void writeLongBackRefCommand(SafeWriter* w, int offset, int len) {
  while (len>0) {
    int thisLen=MIN(len,16);
    w->writeC(0x60|(thisLen-1));
    w->writeC(offset);
    offset=(offset+thisLen)&0xff;
    len-=thisLen;
  }
}

static void writeWait(SafeWriter* w, int& lastWait, int& newWait, bool secret) {
  if (newWait>0) {
    if (secret) {
      while (newWait>0) {
        int thisWait=MIN(newWait,256);
        if (thisWait==lastWait) {
          w->writeC(0);
        } else if (thisWait<=15) {
          w->writeC(thisWait);
        } else {
          w->writeC(0x70);
          w->writeC(thisWait&0xff);
        }
        lastWait=thisWait;
        newWait-=thisWait;
      }
    } else {
      while (newWait>0) {
        w->writeC(0);
        newWait--;
      }
    }
  }
}

static inline bool isFmFreqHReg(int addr) {
  switch (addr) {
    case 0xa4:
    case 0xa5:
    case 0xa6:
    case 0xac:
    case 0xad:
    case 0xae:
    case 0x1a4:
    case 0x1a5:
    case 0x1a6: return true;
    default: return false;
  }
}

void DivExportXGM::run() {
  SafeWriter* w;

  // config
  bool secret=conf.getBool("secret",false);
  bool loop=conf.getBool("loop",false);
  int trailingTicks=conf.getInt("trailingTicks",-1);
  int psgSys=conf.getInt("sysToExportPSG",-1);
  int fmSys=conf.getInt("sysToExportFM",-1);

  e->stop();
  e->repeatPattern=false;
  e->shallStop=false;
  e->setOrder(0);
  e->synchronizedSoft([this, &w, secret, loop, trailingTicks, psgSys, fmSys]() {
    // determine engine rate from chip clocks
    double origRate=e->got.rate;
    double xgmRate=COLOR_NTSC*15/7/144/1024;
    if (fmSys>=0) {
      xgmRate=(double)e->disCont[fmSys].dispatch->chipClock/144/1024;
    } else if (psgSys>=0) {
      xgmRate=(double)e->disCont[psgSys].dispatch->chipClock*15/7/144/1024;
    }
    if (secret) xgmRate*=2;
    e->got.rate=xgmRate;

    // determine loop point
    int loopOrder=0;
    int loopRow=0;
    int loopEnd=0;
    e->walkSong(loopOrder,loopRow,loopEnd);
    logAppendf("loop point: %d %d",loopOrder,loopRow);

    // reset the playback state
    e->curOrder=0;
    e->freelance=false;
    e->playing=false;
    e->extValuePresent=false;
    e->remainingLoops=-1;

    // first, render samples
    w=new SafeWriter;
    w->init();
    w->write("XGM ",4); // header
    for (int i=0; i<64; i++) {
      // will be written later, fill with empty entries for now
      w->writeI(secret?0xffffffff:0x0001ffff);
    }
    int maxSamples=secret?126:63;
    int sampCnt=MIN(e->song.sampleLen,maxSamples);
    size_t sampOffsets[127];
    std::vector<signed char> sampData;
    int bankOffset=0x100;
    for (int i=0; i<sampCnt; i++) {
      if (mustAbort) {
        logAppend("aborted!");
        failed=true;
        running=false;
        return;
      }
      logAppendf("rendering sample #%d",i);
      progress[0].amount=i/(float)sampCnt;
      DivSample* samp=e->getSample(i);
      // TODO resampling
      sampOffsets[i]=w->tell()-0x104;
      if (samp->length8==0) continue;
      if (secret) {
        auto ssdsamp=encode_ssdpcm2(samp->data8,samp->length8,bankOffset);
        w->write(ssdsamp.data(),ssdsamp.size());
        bankOffset+=ssdsamp.size();
      } else {
        w->write(samp->data8,samp->length8);
        bankOffset+=samp->length8;
      }
      // pad to one page
      while ((bankOffset&0xff)!=0) {
        w->writeC(0);
        bankOffset++;
      }
    }
    size_t curPos=w->tell();
    sampOffsets[sampCnt]=curPos-0x104;
    // write sample addresses
    w->seek(4,SEEK_SET);
    for (int i=0; i<sampCnt; i++) {
      size_t len=sampOffsets[i+1]-sampOffsets[i];
      if (secret) {
        w->writeS(len==0?0xffff:sampOffsets[i]/0x100);
      } else {
        w->writeS(len==0?0xffff:sampOffsets[i]/0x100);
        w->writeS(len==0?1:len/0x100);
      }
    }
    // write remaining headers
    w->seek(0x100,SEEK_SET);
    w->writeS((curPos-0x104)/0x100); // sample data size
    w->writeC(secret?0xff:1); // version
    w->writeC(0); // region/tags, TODO
    w->seek(curPos,SEEK_SET);
    progress[0].amount=1.f;

    // then, render song data
    w->writeI(0); // song data size, will be written later
    curPos=w->tell();

    // Prepare to write song data
    unsigned char regs[2][0x200];
    XGMBackRef backref;
    int lastWait=2;
    unsigned char psgChan=0;
    int totalWait=0;

    bool writeLoop=false;
    bool alreadyWroteLoop=false;
    bool done=false;
    int loopPos=-1;
    int loopTickSong=-1;
    int songTick=0;
    int fracWait=0; // accumulates fractional ticks
    std::vector<size_t> tickPos;
    std::vector<int> tickSample;
    bool trailing=false;
    bool beenOneLoopAlready=false;
    int countDown=MAX(0,trailingTicks)+1;

    if (psgSys>=0) e->disCont[psgSys].dispatch->toggleRegisterDump(true);
    if (fmSys>=0) e->disCont[fmSys].dispatch->toggleRegisterDump(true);
    // TODO does XGM require a song to initialize all registers to 0?
    memset(regs,0,sizeof(regs));
    e->playSub(false);

    while (!done) {
      if (mustAbort) {
        logAppend("aborted!");
        failed=true;
        running=false;
        return;
      }
      if (loopPos==-1) {
        if (loopOrder==e->curOrder && loopRow==e->curRow) {
          if ((e->ticks-((e->tempoAccum+e->virtualTempoN)/e->virtualTempoD))<=0) {
            writeLoop=true;
          }
        }
      }
      songTick++;
      tickPos.push_back(w->tell());
      if (e->nextTick(false,true)) {
        if (trailing) beenOneLoopAlready=true;
        trailing=true;
        if (!loop) countDown=0;
        for (int i=0; i<e->chans; i++) {
          if (e->dispatchOfChan[i]!=psgSys && e->dispatchOfChan[i]!=fmSys) continue;
          e->chan[i].wentThroughNote=false;
        }
      }
      if (trailing) {
        switch (trailingTicks) {
          case -1: { // automatic
            bool stillHaveTo=false;
            for (int i=0; i<e->chans; i++) {
              if (e->dispatchOfChan[i]!=psgSys && e->dispatchOfChan[i]!=fmSys) continue;
              if (e->chan[i].goneThroughNote) continue;
              if (e->chan[i].wentThroughNote) {
                stillHaveTo=true;
                break;
              }
            }
            if (!stillHaveTo) countDown=0;
            break;
          }
          case -2: // one loop
            break;
          default: // custom
            countDown--;
            break;
        }
        if (e->song.loopModality!=2) countDown=0;

        if (countDown>0 && !beenOneLoopAlready) {
          loopTickSong++;
        }
      }
      if (countDown<=0 || !e->playing || beenOneLoopAlready) {
        done=true;
        if (!loop) {
          for (int i=0; i<e->song.systemLen; i++) {
            e->disCont[i].dispatch->getRegisterWrites().clear();
          }
          break;
        }
        if (!e->playing) {
          writeLoop=false;
          loopPos=-1;
        }
      }
      // get register dumps and write to new regs for deduplication
      std::vector<unsigned char> psgWrites;
      std::vector<unsigned char> fm0Writes;
      std::vector<unsigned char> fm1Writes;
      std::vector<unsigned char> keyWrites;
      int fmFreqWrites=0;
      int hasSample=-1;
      if (psgSys>=0) {
        std::vector<DivRegWrite>& writes=e->disCont[psgSys].dispatch->getRegisterWrites();
        for (const auto& i: writes) {
          if (i.addr==0) {
            // freq hi writes need a channel select
            int addr=0x12;
            if (i.val&0x80) {
              psgChan=(i.val>>5)&3;
              addr=(i.val&0x10)?0x10:0x11;
            }
            regs[1][psgChan*4+addr]=i.val&0xff;
          }
        }
        writes.clear();
      }
      if (fmSys>=0) {
        std::vector<DivRegWrite>& writes=e->disCont[fmSys].dispatch->getRegisterWrites();
        for (const auto& i: writes) {
          if (i.addr==0x28) keyWrites.push_back(i.val&0xff);
          else if (i.addr==0xffff0000 && i.val<(unsigned int)maxSamples) hasSample=i.val+1;
          else if (i.addr==0xffff0002) hasSample=0;
          else if (i.addr>=0x22 && i.addr<0x200) regs[1][i.addr]=i.val&0xff;
        }
        writes.clear();
      }
      std::vector<XGMChangedReg> changedRegs;
      // check for dependent registers and mark them to also change
      for (int i=0; i<4; i++) { // PSG freq hi
        int j=i*4+0x12;
        if (regs[1][j]!=regs[0][j]) regs[0][j-1]=regs[1][j-1]+1;
      }
      for (int i=0; i<9; i++) { // FM freq hi
        int j=FM_FREQLS[i];
        if (regs[1][j]!=regs[0][j] || regs[1][j+4]!=regs[0][j+4]) {
          regs[0][j]=regs[1][j]+1;
          regs[0][j+4]=regs[1][j+4]+1;
        }
      }
      for (int i=0; i<(int)(sizeof(REG_ORDER)/sizeof(REG_ORDER[0])); i++) {
        int j=REG_ORDER[i];
        if (regs[1][j]!=regs[0][j]) {
          // no match offset can be 0 because it's only used in the single
          // match compression where the offset can't be that early
          changedRegs.emplace_back(j,regs[1][j],0);
        }
      }

      // write wait
      if (!changedRegs.empty() || !keyWrites.empty() || hasSample>=0) {
        writeWait(w,lastWait,totalWait,secret);
      }
      
      // write commands
      if (secret) {
        int changedRegsSize=changedRegs.size();
        // long matches
        while (changedRegsSize>2) {
          // reg write order here can be random as long as freq pairs are intact
          int bufMatches[256];
          for (int i=0; i<256; i++) {
            const DivRegWrite& b=backref[i];
            int j=changedRegsSize;
            if (b.addr!=0x28 && !(b.addr==0 && b.val==0)) {
              for (j=0; j<changedRegsSize; j++) {
                short caddr=changedRegs[j].addr;
                if (caddr<0) continue; // matched
                if (caddr<0x20) caddr=0; // PSG has addr 0 in backref
                if ((int)b.addr==caddr && (int)b.val==changedRegs[j].val) break;
              }
            }
            bufMatches[i]=j;
          }
          int maxLen=0;
          int maxPos=0;
          int curLen=0;
          bool denyPSGPair=true;
          bool denyFMPair=true;
          for (int i=0; i<511; i++) {
            bool end=false;
            if (bufMatches[i&0xff]<changedRegsSize) {
              const int mi=bufMatches[i&0xff];
              const XGMChangedReg& c=changedRegs[mi];
              // if it's freq pair, both must have a match
              // PSG
              if (c.addr<0x20) {
                if (
                  (c.addr&3)==1 && (mi+1)<changedRegsSize &&
                  changedRegs[mi+1].addr==(c.addr+1)
                ) {
                  denyPSGPair=changedRegs[mi+1].val!=backref[(i+1)&0xff].val;
                } else if ((c.addr&3)==2) {
                  end=denyPSGPair;
                  denyPSGPair=true;
                }
              }
              // FM
              if (isFmFreqHReg(c.addr)) {
                end=(
                  bufMatches[(i+1)&0xff]>=changedRegsSize ||
                  (mi+1)>=changedRegsSize || changedRegs[mi+1].addr!=c.addr+4
                );
              } else if (isFmFreqHReg(c.addr+4)) {
                end=denyFMPair;
                denyFMPair=true;
              }
            } else {
              end=true;
            }
            if (end) {
              if ((i-curLen)>=256) break;
              if (curLen>maxLen) {
                maxLen=curLen;
                maxPos=i-curLen;
              }
              curLen=0;
              denyFMPair=true;
              denyPSGPair=true;
            } else {
              curLen++;
            }
          }
          if (maxLen<3) break;
          writeLongBackRefCommand(w,(maxPos-backref.pos)&0xff,maxLen);
          for (int i=0; i<maxLen; i++) {
            changedRegs[bufMatches[(maxPos+i)&0xff]].addr=-1;
          }
        }
        // single matches
        // find matches first
        for (int i=0; i<changedRegsSize; i++) {
          short addr=changedRegs[i].addr;
          unsigned char val=changedRegs[i].val;
          if (addr<0) { // matched
            changedRegs[i].match=0;
            continue;
          }
          if (addr<0x20) addr=0; // PSG has addr 0 in backref
          int j;
          for (j=128; j<256; j++) {
            const auto& b=backref.rel(j);
            if (addr==(int)b.addr && val==(int)b.val) break;
          }
          changedRegs[i].match=j&0xff;
        }
        // then put them with some ordering restrictions
        for (int i=0; i<changedRegsSize; i++) {
          short addr=changedRegs[i].addr;
          unsigned char match=changedRegs[i].match;
          if (match==0) continue;
          // if it's freq pair, both must have a match
          // PSG
          if (
            addr<0x20 && (addr&3)==1 && (i+1)<(int)changedRegs.size() &&
            changedRegs[i+1].addr==(addr+1)
          ) {
            if (changedRegs[i+1].match!=0) {
              w->writeC(match);
              w->writeC(changedRegs[i+1].match);
              changedRegs[i].addr=-1;
              changedRegs[i+1].addr=-1;
            }
            i++; // skip next one
            continue;
          }
          // FM
          if (isFmFreqHReg(addr)) {
            // i+1 going out of bounds shouldn't happen here
            // or else something is wrong in earlier steps
            if (changedRegs[i+1].match!=0) {
              w->writeC(match);
              w->writeC(changedRegs[i+1].match);
              changedRegs[i].addr=-1;
              changedRegs[i+1].addr=-1;
            }
            i++; // skip next one
            continue;
          }
          // disallow lone lower freq matches
          if (!(addr<0x20 && (addr&3)==2) && !isFmFreqHReg(addr+4)) {
            w->writeC(match);
            changedRegs[i].addr=-1;
          }
        }
      }
      for (const XGMChangedReg& i: changedRegs) {
        if (i.addr<0) {
          // matched
          continue;
        } else if (i.addr<0x20) {
          psgWrites.push_back(i.val);
        } else {
          // separate freq writes out of other regs
          int j=9;
          if (secret) {
            for (j=0; j<9; j++) { // FM freq hi
              if ((i.addr&0x1fb)==FM_FREQLS[j]) break;
            }
          }
          if (j<9) {
            fmFreqWrites|=(0x100>>j);
          } else if (i.addr<0x100) {
            fm0Writes.push_back(i.addr&0xff);
            fm0Writes.push_back(i.val);
          } else {
            fm1Writes.push_back(i.addr&0xff);
            fm1Writes.push_back(i.val);
          }
        }
      }
      // play sample first, as this is the most delayed
      if (hasSample>=0) {
        w->writeC(0x50);
        w->writeC(hasSample);
      }
      writeCommand(0x10,w,psgWrites,0,psgWrites.size(),true,0x00,backref);
      writeCommand(0x20,w,fm0Writes,0,fm0Writes.size()/2,false,0x00,backref);
      writeCommand(0x30,w,fm1Writes,0,fm1Writes.size()/2,false,0x100,backref);
      // write fm frequencies
      if (fmFreqWrites>0) {
        w->writeS_BE(0x7200|fmFreqWrites);
        for (int i=0; i<9; i++) {
          if (fmFreqWrites&(0x100>>i)) {
            int addr=FM_FREQLS[i];
            backref.add(addr+4,regs[1][addr+4]);
            backref.add(addr,regs[1][addr]);
            w->writeC(regs[1][addr+4]);
            w->writeC(regs[1][addr]);
          }
        }
      }
      // write fm keys with compression
      if (secret && keyWrites.size()>2) {
        const unsigned char* raw=keyWrites.data();
        int size=(int)keyWrites.size();
        int i=0;
        int last=0;
        while (i<size) {
          int longest=0;
          int longestPos=0;
          // j doesn't start at 0 here to avoid overwrites
          for (int j=size; j<256; j++) {
            int bpos=j;
            int k=0;
            while (i+k<size && backref.rel(bpos).addr==0x28 && backref.rel(bpos).val==raw[i+k]) {
              bpos=(bpos+1)&0xff;
              k++;
            }
            if (k>longest) {
              longest=k;
              longestPos=j;
            }
          }
          if (longest>2) {
            writeCommand(0x40,w,keyWrites,last,i-last,true,0x28,backref);
            writeLongBackRefCommand(w,(longestPos+last-i)&0xff,longest);
            last=i+longest;
          }
          i+=MAX(longest,1);
        }
        writeCommand(0x40,w,keyWrites,last,size-last,true,0x28,backref);
      } else {
        writeCommand(0x40,w,keyWrites,0,keyWrites.size(),true,0x28,backref);
      }

      // done, move all regs to last
      memcpy(regs[0],regs[1],sizeof(regs[0]));

      // write wait
      totalWait+=e->cycles>>MASTER_CLOCK_PREC;
      fracWait+=e->cycles&MASTER_CLOCK_MASK;
      totalWait+=fracWait>>MASTER_CLOCK_PREC;
      fracWait&=MASTER_CLOCK_MASK;
      if (writeLoop && !alreadyWroteLoop) {
        writeWait(w,lastWait,totalWait,secret);
        writeLoop=false;
        alreadyWroteLoop=true;
        loopPos=w->tell();
        loopTickSong=songTick;
      }

      // TODO finer progress[1] update once song position and length API becomes easier
    }
    // end of song
    writeWait(w,lastWait,totalWait,secret);
    if (loop) {
      if (loopPos==-1) {
        w->writeC(0x7f);
      } else if (loopTickSong<0 || loopTickSong>(int)tickPos.size()) {
        logW("loopTickSong out of range! %d>%d",loopTickSong,(int)tickPos.size());
        w->writeC(0x7f);
      } else {
        size_t realLoopPos=tickPos[loopTickSong];
        logI("realLoopPos: %d",realLoopPos);
        w->writeI(((realLoopPos-curPos)<<8)|0x7e);
      }
    } else {
      w->writeC(0x7f);
    }
    size_t songSize=w->tell()-curPos;
    w->seek(curPos-4,SEEK_SET);
    w->writeI(songSize);
    progress[1].amount=1.f;

    // done - close out.
    e->got.rate=origRate;
    if (psgSys>=0) e->disCont[psgSys].dispatch->toggleRegisterDump(false);
    if (fmSys>=0) e->disCont[fmSys].dispatch->toggleRegisterDump(false);

    e->remainingLoops=-1;
    e->playing=false;
    e->freelance=false;
    e->extValuePresent=false;
  });

  progress[0].amount=1.0f;

  logAppend("finished!");

  output.emplace_back("out.xgm",w);
  running=false;
}

bool DivExportXGM::go(DivEngine* eng) {
  progress[0].name="Samples";
  progress[0].amount=0.0f;
  progress[1].name="Song Data";
  progress[1].amount=0.0f;

  e=eng;
  running=true;
  failed=false;
  mustAbort=false;
  exportThread=new std::thread(&DivExportXGM::run,this);
  return true;
}

void DivExportXGM::wait() {
  if (exportThread!=NULL) {
    exportThread->join();
    delete exportThread;
  }
}

void DivExportXGM::abort() {
  mustAbort=true;
  wait();
}

bool DivExportXGM::isRunning() {
  return running;
}

bool DivExportXGM::hasFailed() {
  return failed;
}

DivROMExportProgress DivExportXGM::getProgress(int index) {
  if (index<0 || index>2) return progress[2];
  return progress[index];
}
