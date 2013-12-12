
/* SSDV - Slow Scan Digital Video                                        */
/*=======================================================================*/
/* Copyright 2011-2012 Philip Heron <phil@sanslogic.co.uk                */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */
/* Modified by VK3TBC to compile under Arduino environment 13/09/2013    */
/* Also added the ability to get image data from serial camera           */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ssdv.h"
#include "rs8.h"
#include "camera.h"
#include <arduino.h>
/* Recognised JPEG markers */
enum {
  J_TEM = 0xFF01,
  J_SOF0 = 0xFFC0, J_SOF1,  J_SOF2,  J_SOF3,  J_DHT,   J_SOF5,  J_SOF6, J_SOF7,
  J_JPG,  J_SOF9,  J_SOF10, J_SOF11, J_DAC,   J_SOF13, J_SOF14, J_SOF15,
  J_RST0, J_RST1,  J_RST2,  J_RST3,  J_RST4,  J_RST5,  J_RST6,  J_RST7,
  J_SOI,  J_EOI,   J_SOS,   J_DQT,   J_DNL,   J_DRI,   J_DHP,   J_EXP,
  J_APP0, J_APP1,  J_APP2,  J_APP3,  J_APP4,  J_APP5,  J_APP6,  J_APP7,
  J_APP8, J_APP9,  J_APP10, J_APP11, J_APP12, J_APP13, J_APP14, J_APP15,
  J_JPG0, J_JPG1,  J_JPG2,  J_JPG3,  J_JPG4,  J_JPG5,  J_JPG6,  J_SOF48,
  J_LSE,  J_JPG9,  J_JPG10, J_JPG11, J_JPG12, J_JPG13, J_COM,
} 
jpeg_marker_t;

/* APP0 header data */
static const uint8_t app0[14] = {
  0x4A,0x46,0x49,0x46,0x00,0x01,0x01,0x01,0x00,0x48,0x00,0x48,0x00,0x00,
};

/* SOS header data */
static const uint8_t sos[10] = {
  0x03,0x01,0x00,0x02,0x11,0x03,0x11,0x00,0x3F,0x00,
};

/* Quantisation tables */
static const uint8_t std_dqt0[65] = {
  0x00,0x10,0x0C,0x0C,0x0E,0x0C,0x0A,0x10,0x0E,0x0E,0x0E,0x12,0x12,0x10,0x14,0x18,
  0x28,0x1A,0x18,0x16,0x16,0x18,0x32,0x24,0x26,0x1E,0x28,0x3A,0x34,0x3E,0x3C,0x3A,
  0x34,0x38,0x38,0x40,0x48,0x5C,0x4E,0x40,0x44,0x58,0x46,0x38,0x38,0x50,0x6E,0x52,
  0x58,0x60,0x62,0x68,0x68,0x68,0x3E,0x4E,0x72,0x7A,0x70,0x64,0x78,0x5C,0x66,0x68,
  0x64,
};

static const uint8_t std_dqt1[65] = {
  0x01,0x12,0x12,0x12,0x16,0x16,0x16,0x30,0x1A,0x1A,0x30,0x64,0x42,0x38,0x42,0x64,
  0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,
  0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,
  0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,0x64,
  0x64,
};

/* Standard Huffman tables */
static const uint8_t std_dht00[29] = {
  0x00,0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
};

static const uint8_t std_dht01[29] = {
  0x01,0x00,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,
  0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
};

static const uint8_t std_dht10[179] = {
  0x10,0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,0x01,
  0x7D,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,
  0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,
  0xF0,0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,
  0x28,0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,
  0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,
  0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,
  0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,
  0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,
  0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,
  0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,
  0xF8,0xF9,0xFA,
};

static const uint8_t std_dht11[179] = {
  0x11,0x00,0x02,0x01,0x02,0x04,0x04,0x03,0x04,0x07,0x05,0x04,0x04,0x00,0x01,0x02,
  0x77,0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,
  0x71,0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,
  0xF0,0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,
  0x26,0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,
  0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,
  0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,
  0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,
  0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,
  0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,
  0xDA,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,
  0xF8,0xF9,0xFA,
};

/* Helper for returning the current DHT table */
#define SDHT (s->sdht[s->acpart ? 1 : 0][s->component ? 1 : 0])
#define DDHT (s->ddht[s->acpart ? 1 : 0][s->component ? 1 : 0])

/* Helpers for looking up the current DQT value */
#define SDQT (s->sdqt[s->component ? 1 : 0][1 + s->acpart])
#define DDQT (s->ddqt[s->component ? 1 : 0][1 + s->acpart])

/* Helpers for converting between DQT tables */
#define AADJ(i) (SDQT == DDQT ? (i) : irdiv(i, DDQT))
#define UADJ(i) (SDQT == DDQT ? (i) : (i * SDQT))
#define BADJ(i) (SDQT == DDQT ? (i) : irdiv(i * SDQT, DDQT))

/* Integer-only division with rounding */
static int irdiv(int i, int div)
{
  i = i * 2 / div;
  if(i & 1) i += (i > 0 ? 1 : -1);
  return(i / 2);
}

/*
static char *strbits(uint32_t value, uint8_t bits)
 {
 	static char s[33];
 	char *ss = s;
 	while(bits--) *(ss++) = value & 1 << bits ? '1' : '0';
 	*ss = '\0';
 	return(s);
 }
 */

static void *stblcpy(ssdv_t *s, const void *src, size_t n)
{
  void *r;
  if(s->stbl_len + n > TBL_LEN + HBUFF_LEN) return(NULL);
  r = memcpy(&s->stbls[s->stbl_len], src, n);
  s->stbl_len += n;
  return(r);
}

static void *dtblcpy(ssdv_t *s, const uint8_t *src, size_t n)
{
  void *r;
  if(s->dtbl_len + n > TBL_LEN) return(NULL);
  r = memcpy(&s->dtbls[s->dtbl_len], src, n);
  s->dtbl_len += n;
  return(r);
}

static uint32_t crc32(uint8_t *data, size_t length)
{
  uint32_t crc, x;
  uint8_t i, *d;

  for(d = data, crc = 0xFFFFFFFF; length; length--)
  {
    x = (crc ^ *(d++)) & 0xFF;
    for(i = 8; i > 0; i--)
    {
      if(x & 1) x = (x >> 1) ^ 0xEDB88320;
      else x >>= 1;
    }
    crc = (crc >> 8) ^ x;
  }

  return(crc ^ 0xFFFFFFFF);
}

static uint32_t encode_callsign(char *callsign)
{
  uint32_t x;
  char *c;

  /* Point c at the end of the callsign */
  for(c = callsign; *c; c++);

  /* Encode it backwards */
  x = 0;
  for(c--; c >= callsign; c--)
  {
    x *= 40;
    if(*c >= 'A' && *c <= 'Z') x += *c - 'A' + 14;
    else if(*c >= 'a' && *c <= 'z') x += *c - 'a' + 14;
    else if(*c >= '0' && *c <= '9') x += *c - '0' + 1;
  }

  return(x);
}

static char *decode_callsign(char *callsign, uint32_t code)
{
  char *c, s;

  *callsign = '\0';

  /* Is callsign valid? */
  if(code > 0xF423FFFF) return(callsign);

  for(c = callsign; code; c++)
  {
    s = code % 40;
    if(s == 0) *c = '-';
    else if(s < 11) *c = '0' + s - 1;
    else if(s < 14) *c = '-';
    else *c = 'A' + s - 14;
    code /= 40;
  }
  *c = '\0';

  return(callsign);
}

static char jpeg_dht_lookup(ssdv_t *s, uint8_t *symbol, uint8_t *width)
{
  uint16_t code = 0;
  uint8_t cw, n;
  uint8_t *dht, *ss;

  /* Select the appropriate huffman table */
  dht = SDHT;
  ss = &dht[17];

  for(cw = 1; cw <= 16; cw++)
  {
    /* Got enough bits? */
    if(cw > s->worklen) return(SSDV_FEED_ME);

    /* Compare against each code 'cw' bits wide */
    for(n = dht[cw]; n > 0; n--)
    {
      if(s->workbits >> (s->worklen - cw) == code)
      {
        /* Found a match */
        *symbol = *ss;
        *width = cw;
        return(SSDV_OK);
      }
      ss++; 
      code++;
    }

    code <<= 1;
  }

  /* No match found - error */
  return(SSDV_ERROR);
}

static char jpeg_dht_lookup_symbol(ssdv_t *s, uint8_t symbol, uint16_t *bits, uint8_t *width)
{
  uint16_t code = 0;
  uint8_t cw, n;
  uint8_t *dht, *ss;

  dht = DDHT;
  ss = &dht[17];

  for(cw = 1; cw <= 16; cw++)
  {
    for(n = dht[cw]; n > 0; n--)
    {
      if(*ss == symbol)
      {
        /* Found a match */
        *bits = code;
        *width = cw;
        return(SSDV_OK);
      }
      ss++; 
      code++;
    }

    code <<= 1;
  }

  /* No match found - error */
  return(SSDV_ERROR);
}

static int jpeg_int(int bits, int width)
{
  int b = (1 << width) - 1;
  if(bits <= b >> 1) bits = -(bits ^ b);
  return(bits);
}

static void jpeg_encode_int(int value, int *bits, uint8_t *width)
{
  *bits = value;

  /* Calculate the number of bits */
  if(value < 0) value = -value;
  for(*width = 0; value; value >>= 1) (*width)++;

  /* Fix negative values */
  if(*bits < 0) *bits = -*bits ^ ((1 << *width) - 1);
}

/*****************************************************************************/

static char ssdv_outbits(ssdv_t *s, uint16_t bits, uint8_t length)
{
  uint8_t b;

  if(length)
  {
    s->outbits <<= length;
    s->outbits |= bits & ((1 << length) - 1);
    s->outlen += length;
  }

  while(s->outlen >= 8 && s->out_len > 0)
  {
    b = s->outbits >> (s->outlen - 8);

    /* Put the byte into the output buffer */
    *(s->outp++) = b;
    s->outlen -= 8;
    s->out_len--;

    /* Insert stuffing byte if needed */
    if(s->out_stuff && b == 0xFF)
    {
      s->outbits &= (1 << s->outlen) - 1;
      s->outlen += 8;
    }
  }

  return(s->out_len ? SSDV_OK : SSDV_BUFFER_FULL);
}

static char ssdv_outbits_sync(ssdv_t *s)
{
  uint8_t b = s->outlen % 8;
  if(b) return(ssdv_outbits(s, 0xFF, 8 - b));
  return(SSDV_OK);
}

static char ssdv_out_jpeg_int(ssdv_t *s, uint8_t rle, int value)
{
  uint16_t huffbits = 0;
  int intbits;
  uint8_t hufflen = 0, intlen;
  int r;

  jpeg_encode_int(value, &intbits, &intlen);
  r = jpeg_dht_lookup_symbol(s, (rle << 4) | (intlen & 0x0F), &huffbits, &hufflen);

  if(r != SSDV_OK) fprintf(stderr, "jpeg_dht_lookup_symbol: %i (%i:%i)\n", r, value, rle);

  ssdv_outbits(s, huffbits, hufflen);
  if(intlen) ssdv_outbits(s, intbits, intlen);

  return(SSDV_OK);
}

static char ssdv_process(ssdv_t *s)
{
  if(s->state == S_HUFF)//changed
  {
    uint8_t symbol, width;
    int r;

    /* Lookup the code, return if error or not enough bits yet */
    if((r = jpeg_dht_lookup(s, &symbol, &width)) != SSDV_OK)
      return(r);

    if(s->acpart == 0) /* DC */
    {
      if(symbol == 0x00)
      {
        /* No change in DC from last block */
        if(s->reset_mcu == s->mcu_id && (s->mcupart == 0 || s->mcupart >= s->ycparts))
        {
          if(s->mode == S_ENCODING) ssdv_out_jpeg_int(s, 0, s->adc[s->component]);
          else
          {
            ssdv_out_jpeg_int(s, 0, 0 - s->dc[s->component]);
            s->dc[s->component] = 0;
          }
        }
        else ssdv_out_jpeg_int(s, 0, 0);

        /* skip to the next AC part immediately */
        s->acpart++;
      }
      else
      {
        /* DC value follows, 'symbol' bits wide */
        s->state =    S_INT;
        s->needbits = symbol;
      }
    }
    else /* AC */
    {
      s->acrle = 0;
      if(symbol == 0x00)
      {
        /* EOB -- all remaining AC parts are zero */
        ssdv_out_jpeg_int(s, 0, 0);
        s->acpart = 64;
      }
      else if(symbol == 0xF0)
      {
        /* The next 16 AC parts are zero */
        ssdv_out_jpeg_int(s, 15, 0);
        s->acpart += 16;
      }
      else
      {
        /* Next bits are an integer value */
        s->state = S_INT;
        s->acrle = symbol >> 4;
        s->acpart += s->acrle;
        s->needbits = symbol & 0x0F;
      }
    }

    /* Clear processed bits */
    s->worklen -= width;
    s->workbits &= (1 << s->worklen) - 1;
  }
  else if(s->state == S_INT)
  {
    int i;

    /* Not enough bits yet? */
    if(s->worklen < s->needbits) return(SSDV_FEED_ME);

    /* Decode the integer */
    i = jpeg_int(s->workbits >> (s->worklen - s->needbits), s->needbits);

    if(s->acpart == 0) /* DC */
    {
      if(s->reset_mcu == s->mcu_id && (s->mcupart == 0 || s->mcupart >= s->ycparts))
      {
        if(s->mode == S_ENCODING)
        {
          /* Output absolute DC value */
          s->dc[s->component] += UADJ(i);
          s->adc[s->component] = AADJ(s->dc[s->component]);
          ssdv_out_jpeg_int(s, 0, s->adc[s->component]);
        }
        else
        {
          /* Output relative DC value */
          ssdv_out_jpeg_int(s, 0, i - s->dc[s->component]);
          s->dc[s->component] = i;
        }
      }
      else
      {
        if(s->mode == S_DECODING)
        {
          s->dc[s->component] += UADJ(i);
          ssdv_out_jpeg_int(s, 0, i);
        }
        else
        {
          /* Output relative DC value */
          s->dc[s->component] += UADJ(i);

          /* Calculate closest adjusted DC value */
          i = AADJ(s->dc[s->component]);
          ssdv_out_jpeg_int(s, 0, i - s->adc[s->component]);
          s->adc[s->component] = i;
        }
      }
    }
    else /* AC */
    {
      if((i = BADJ(i)))
      {
        s->accrle += s->acrle;
        while(s->accrle >= 16)
        {
          ssdv_out_jpeg_int(s, 15, 0);
          s->accrle -= 16;
        }
        ssdv_out_jpeg_int(s, s->accrle, i);
        s->accrle = 0;
      }
      else
      {
        /* AC value got reduced to 0 in the DQT conversion */
        if(s->acpart >= 63)
        {
          ssdv_out_jpeg_int(s, 0, 0);
          s->accrle = 0;
        }
        else s->accrle += s->acrle + 1;
      }
    }

    /* Next AC part to expect */
    s->acpart++;

    /* Next bits are a huffman code */
    s->state = S_HUFF;

    /* Clear processed bits */
    s->worklen -= s->needbits;
    s->workbits &= (1 << s->worklen) - 1;
  }

  if(s->acpart >= 64)
  {
    /* Reached the end of this MCU part */
    if(++s->mcupart == s->ycparts + 2)
    {
      s->mcupart = 0;
      s->mcu_id++;

      /* Test for the end of image */
      if(s->mcu_id >= s->mcu_count)
      {
        /* Flush any remaining bits */
        ssdv_outbits_sync(s);
        return(SSDV_EOI);
      }

      /* Set the packet MCU marker - encoder only */
      if(s->mode == S_ENCODING && s->packet_mcu_id == 0xFFFF)
      {
        /* The first MCU of each packet should be byte aligned */
        ssdv_outbits_sync(s);

        s->reset_mcu = s->mcu_id;
        s->packet_mcu_id = s->mcu_id;
        s->packet_mcu_offset = SSDV_PKT_SIZE_PAYLOAD - s->out_len;
      }

      if(s->mode == S_DECODING && s->mcu_id == s->reset_mcu)
        s->workbits = s->worklen = 0;

      /* Test for a reset marker */
      if(s->dri > 0 && s->mcu_id > 0 && s->mcu_id % s->dri == 0)
      {
        s->state = S_MARKER;
        return(SSDV_FEED_ME);
      }
    }

    if(s->mcupart < s->ycparts) s->component = 0;
    else s->component = s->mcupart - s->ycparts + 1;

    s->acpart = 0;
    s->accrle = 0;
  }

  if(s->out_len == 0) return(SSDV_BUFFER_FULL);

  return(SSDV_OK);
}

/*****************************************************************************/

static void ssdv_memset_prng(uint8_t *s, size_t n)
{
  /* A very simple PRNG for noise whitening */
  uint8_t l = 0x00;
  for(; n > 0; n--) *(s++) = (l = l * 245 + 45);
}

static char ssdv_have_marker(ssdv_t *s)
{
  switch(s->marker)
  {
  case J_SOF0:
  case J_SOS:
  case J_DRI:
  case J_DHT:
  case J_DQT:
    /* Copy the data before processing */
    if(s->marker_len > TBL_LEN + HBUFF_LEN - s->stbl_len)
    {
      /* Not enough memory ... shouldn't happen! */
      return(SSDV_ERROR);
    }

    s->marker_data     = &s->stbls[s->stbl_len];
    s->marker_data_len = 0;
    s->state           = S_MARKER_DATA;
    break;

  case J_SOF2:
    /* Don't do progressive images! */
    fprintf(stderr, "Error: Progressive images not supported\n");
    return(SSDV_ERROR);

  case J_EOI:
    s->state = S_EOI;
    break;

  case J_RST0:
  case J_RST1:
  case J_RST2:
  case J_RST3:
  case J_RST4:
  case J_RST5:
  case J_RST6:
  case J_RST7:
    s->dc[0]  = s->dc[1]  = s->dc[2]  = 0;
    s->mcupart = s->acpart = s->component = 0;
    s->acrle = s->accrle = 0;
    s->workbits = s->worklen = 0;
    s->state = S_HUFF;
    break;

  default:
    /* Ignore other marks, skipping any associated data */
    s->in_skip = s->marker_len;
    s->state   = S_MARKER;
    break;
  }

  return(SSDV_OK);
}

static char ssdv_have_marker_data(ssdv_t *s)
{
  uint8_t *d = s->marker_data;
  size_t l = s->marker_len;
  int i;

  switch(s->marker)
  {
  case J_SOF0:
    s->width  = (d[3] << 8) | d[4];
    s->height = (d[1] << 8) | d[2];

    /* Display information about the image... */
    fprintf(stderr, "Precision: %i\n", d[0]);
    fprintf(stderr, "Resolution: %ix%i\n", s->width, s->height);
    fprintf(stderr, "Components: %i\n", d[5]);

    /* The image must have a precision of 8 */
    if(d[0] != 8)
    {
      fprintf(stderr, "Error: The image must have a precision of 8\n");
      return(SSDV_ERROR);
    }

    /* The image must have 3 components (Y'Cb'Cr) */
    if(d[5] != 3)
    {
      fprintf(stderr, "Error: The image must have 3 components\n");
      return(SSDV_ERROR);
    }

    /* Maximum image is 4080x4080 */
    if(s->width > 4080 || s->height > 4080)
    {
      fprintf(stderr, "Error: The image is too big. Maximum resolution is 4080x4080\n");
      return(SSDV_ERROR);
    }

    /* The image dimensions must be a multiple of 16 */
    if((s->width & 0x0F) || (s->height & 0x0F))
    {
      fprintf(stderr, "Error: The image dimensions must be a multiple of 16\n");
      return(SSDV_ERROR);
    }

    /* TODO: Read in the quantisation table ID for each component */
    // 01 22 00 02 11 01 03 11 01
    for(i = 0; i < 3; i++)
    {
      uint8_t *dq = &d[i * 3 + 6];
      if(dq[0] != i + 1)
      {
        fprintf(stderr, "Error: Components are not in order in the SOF0 header\n");
        return(SSDV_ERROR);
      }

      fprintf(stderr, "DQT table for component %i: %02X, Sampling factor: %ix%i\n", dq[0], dq[2], dq[1] & 0x0F, dq[1] >> 4);

      /* The first (Y) component must have a factor of 2x2,2x1,1x2 or 1x1 */
      if(dq[0] == 1)
      {
        switch(dq[1])
        {
        case 0x22: 
          s->mcu_mode = 0; 
          s->ycparts = 4; 
          break;
        case 0x12: 
          s->mcu_mode = 1; 
          s->ycparts = 2; 
          break;
        case 0x21: 
          s->mcu_mode = 2; 
          s->ycparts = 2; 
          break;
        case 0x11: 
          s->mcu_mode = 3; 
          s->ycparts = 1; 
          break;
        default:
          fprintf(stderr, "Error: Component 1 sampling factor is not supported\n");
          return(SSDV_ERROR);
        }
      }
      else if(dq[0] != 1 && dq[1] != 0x11)
      {
        fprintf(stderr, "Error: Component %i sampling factor must be 1x1\n", dq[0]);
        return(SSDV_ERROR);
      }
    }

    /* Calculate number of MCU blocks in this image */
    switch(s->mcu_mode)
    {
    case 0: 
      l = (s->width >> 4) * (s->height >> 4); 
      break;
    case 1: 
      l = (s->width >> 4) * (s->height >> 3); 
      break;
    case 2: 
      l = (s->width >> 3) * (s->height >> 4); 
      break;
    case 3: 
      l = (s->width >> 3) * (s->height >> 3); 
      break;
    }

    fprintf(stderr, "MCU blocks: %i\n", (int) l);

    if(l > 0xFFFF)
    {
      fprintf(stderr, "Error: Maximum number of MCU blocks is 65535\n");
      return(SSDV_ERROR);
    }

    s->mcu_count = l;

    break;

  case J_SOS:
    fprintf(stderr, "Components: %i\n", d[0]);

    /* The image must have 3 components (Y'Cb'Cr) */
    if(d[0] != 3)
    {
      fprintf(stderr, "Error: The image must have 3 components\n");
      return(SSDV_ERROR);
    }

    for(i = 0; i < 3; i++)
    {
      uint8_t *dh = &d[i * 2 + 1];
      if(dh[0] != i + 1)
      {
        fprintf(stderr, "Error: Components are not in order in the SOF0 header\n");
        return(SSDV_ERROR);
      }

      fprintf(stderr, "Component %i DHT: %02X\n", dh[0], dh[1]);
    }

    /* Do I need to look at the last three bytes of the SOS data? */
    /* 00 3F 00 */

    /* Verify all of the DQT and DHT tables where loaded */
    if(!s->sdqt[0] || !s->sdqt[1])
    {
      fprintf(stderr, "Error: The image is missing one or more DQT tables\n");
      return(SSDV_ERROR);
    }

    if(!s->sdht[0][0] || !s->sdht[0][1] ||
      !s->sdht[1][0] || !s->sdht[1][1])
    {
      fprintf(stderr, "Error: The image is missing one or more DHT tables\n");
      return(SSDV_ERROR);
    }

    /* The SOS data is followed by the image data */
    s->state = S_HUFF;

    return(SSDV_OK);

  case J_DHT:
    s->stbl_len += l;
    while(l > 0)
    {
      int i, j;

      switch(d[0])
      {
      case 0x00: 
        s->sdht[0][0] = d; 
        break;
      case 0x01: 
        s->sdht[0][1] = d; 
        break;
      case 0x10: 
        s->sdht[1][0] = d; 
        break;
      case 0x11: 
        s->sdht[1][1] = d; 
        break;
      }

      /* Skip to the next DHT table */
      for(j = 17, i = 1; i <= 16; i++)
        j += d[i];

      l -= j;
      d += j;
    }
    break;

  case J_DQT:
    s->stbl_len += l;
    while(l > 0)
    {
      switch(d[0])
      {
      case 0x00: 
        s->sdqt[0] = d; 
        break;
      case 0x01: 
        s->sdqt[1] = d; 
        break;
      }

      /* Skip to the next one, if present */
      l -= 65;
      d += 65;
    }
    break;

  case J_DRI:
    s->dri = (d[0] << 8) + d[1];
    fprintf(stderr, "Reset interval: %i blocks\n", s->dri);
    break;
  }

  s->state = S_MARKER;
  return(SSDV_OK);
}

char ssdv_enc_init(ssdv_t *s, char *callsign, uint8_t image_id)
{
  memset(s, 0, sizeof(ssdv_t));
  s->image_id = image_id;
  s->callsign = encode_callsign(callsign);
  s->mode = S_ENCODING;

  /* Prepare the output JPEG tables */
  s->ddqt[0] = (uint8_t*)dtblcpy(s, std_dqt0, sizeof(std_dqt0));
  s->ddqt[1] = (uint8_t*)dtblcpy(s, std_dqt1, sizeof(std_dqt1));
  s->ddht[0][0] = (uint8_t*)dtblcpy(s, std_dht00, sizeof(std_dht00));
  s->ddht[0][1] = (uint8_t*)dtblcpy(s, std_dht01, sizeof(std_dht01));
  s->ddht[1][0] = (uint8_t*)dtblcpy(s, std_dht10, sizeof(std_dht10));
  s->ddht[1][1] = (uint8_t*)dtblcpy(s, std_dht11, sizeof(std_dht11));

  return(SSDV_OK);
}

char ssdv_enc_set_buffer(ssdv_t *s, uint8_t *buffer)
{
  s->out     = buffer;
  s->outp    = buffer + SSDV_PKT_SIZE_HEADER;
  s->out_len = SSDV_PKT_SIZE_PAYLOAD;

  /* Zero the payload memory */
  memset(s->out, 0, SSDV_PKT_SIZE);

  /* Flush the output bits */
  ssdv_outbits(s, 0, 0);

  return(SSDV_OK);
}

char ssdv_enc_get_packet(ssdv_t *s)
{
  int r;
  uint8_t b;

  /* Have we reached the end of the image? */
  if(s->state == S_EOI) return(SSDV_EOI);

  /* If the output buffer is empty, re-initialise */
  if(s->out_len == 0) ssdv_enc_set_buffer(s, s->out);

  while(s->in_len)
  {
    b = *(s->inp++);
    s->in_len--;

    /* Skip bytes if necessary */
    if(s->in_skip) { 
      s->in_skip--; 
      continue; 
    }

    switch(s->state)
    {
    case S_MARKER:
      s->marker = (s->marker << 8) | b;

      if(s->marker == J_TEM ||
        (s->marker >= J_RST0 && s->marker <= J_EOI))
      {
        /* Marker without data */
        s->marker_len = 0;
        r = ssdv_have_marker(s);
        if(r != SSDV_OK) return(r);
      }
      else if(s->marker >= J_SOF0 && s->marker <= J_COM)
      {
        /* All other markers are followed by data */
        s->marker_len = 0;
        s->state = S_MARKER_LEN;
        s->needbits = 16;
      }
      break;

    case S_MARKER_LEN:
      s->marker_len = (s->marker_len << 8) | b;
      if((s->needbits -= 8) == 0)
      {
        s->marker_len -= 2;
        r = ssdv_have_marker(s);
        if(r != SSDV_OK) return(r);
      }
      break;

    case S_MARKER_DATA:
      s->marker_data[s->marker_data_len++] = b;
      if(s->marker_data_len == s->marker_len)
      {
        r = ssdv_have_marker_data(s);
        if(r != SSDV_OK) return(r);
      }
      break;

    case S_HUFF:
    case S_INT:
      /* Is the next byte a stuffing byte? Skip it */
      /* TODO: Test the next byte is actually 0x00 */
      if(b == 0xFF) s->in_skip++;

      /* Add the new byte to the work area */
      s->workbits = (s->workbits << 8) | b;
      s->worklen += 8;

      /* Process the new data until more needed, or an error occurs */
      while((r = ssdv_process(s)) == SSDV_OK);

      if(r == SSDV_BUFFER_FULL || r == SSDV_EOI)
      {
        uint16_t mcu_id     = s->packet_mcu_id;
        uint8_t i, mcu_offset = s->packet_mcu_offset;
        uint32_t x;

        if(mcu_offset != 0xFF && mcu_offset >= SSDV_PKT_SIZE_PAYLOAD)
        {
          /* The first MCU begins in the next packet, not this one */
          mcu_id = 0xFFFF;
          mcu_offset = 0xFF;
          s->packet_mcu_offset -= SSDV_PKT_SIZE_PAYLOAD;
        }
        else
        {
          /* Clear the MCU data for the next packet */
          s->packet_mcu_id = 0xFFFF;
          s->packet_mcu_offset = 0xFF;
        }

        /* A packet is ready, create the headers */
        s->out[0]  = 0x55;                /* Sync */
        s->out[1]  = 0x66;                /* Type */
        s->out[2]  = s->callsign >> 24;
        s->out[3]  = s->callsign >> 16;
        s->out[4]  = s->callsign >> 8;
        s->out[5]  = s->callsign;
        s->out[6]  = s->image_id;         /* Image ID */
        s->out[7]  = s->packet_id >> 8;   /* Packet ID MSB */
        s->out[8]  = s->packet_id & 0xFF; /* Packet ID LSB */
        s->out[9]  = s->width >> 4;       /* Width / 16 */
        s->out[10] = s->height >> 4;      /* Height / 16 */
        s->out[11] = s->mcu_mode & 0x03;  /* MCU mode (2 bits) */
        s->out[12] = mcu_offset;          /* Next MCU offset */
        s->out[13] = mcu_id >> 8;         /* MCU ID MSB */
        s->out[14] = mcu_id & 0xFF;       /* MCU ID LSB */

        /* Fill any remaining bytes with noise */
        if(s->out_len > 0) ssdv_memset_prng(s->outp, s->out_len);

        /* Calculate the CRC codes */
        x = crc32(&s->out[1], SSDV_PKT_SIZE_CRCDATA);

        i = 1 + SSDV_PKT_SIZE_CRCDATA;
        s->out[i++] = (x >> 24) & 0xFF;
        s->out[i++] = (x >> 16) & 0xFF;
        s->out[i++] = (x >> 8) & 0xFF;
        s->out[i++] = x & 0xFF;

        /* Generate the RS codes */
        encode_rs_8(&s->out[1], &s->out[i], 0);

        s->packet_id++;

        /* Have we reached the end of the image data? */
        if(r == SSDV_EOI) s->state = S_EOI;

        return(SSDV_OK);
      }
      else if(r != SSDV_FEED_ME)
      {
        /* An error occured */
        fprintf(stderr, "ssdv_process() failed: %i\n", r);
        return(SSDV_ERROR);
      }
      break;

    case S_EOI:
      /* Shouldn't reach this point */
      break;
    }
  }

  /* Need more data */
  return(SSDV_FEED_ME);
}

char ssdv_enc_feed(ssdv_t *s, uint8_t *buffer, size_t length)
{
  s->inp    = buffer;
  s->in_len = length;
  return(SSDV_OK);
}

/*****************************************************************************/

static void ssdv_write_marker(ssdv_t *s, uint16_t id, uint16_t length, const uint8_t *data)
{
  ssdv_outbits(s, id, 16);
  if(length > 0)
  {
    ssdv_outbits(s, length + 2, 16);
    while(length--) ssdv_outbits(s, *(data++), 8);
  }
}

static void ssdv_out_headers(ssdv_t *s)
{
  uint8_t *b = &s->stbls[s->stbl_len];

  ssdv_write_marker(s, J_SOI,    0, 0);
  ssdv_write_marker(s, J_APP0,  14, app0);
  ssdv_write_marker(s, J_DQT,   65, std_dqt0);  /* DQT Luminance       */
  ssdv_write_marker(s, J_DQT,   65, std_dqt1);  /* DQT Chrominance     */

  /* Build SOF0 header */
  b[0]  = 8; /* Precision */
  b[1]  = s->height >> 8;
  b[2]  = s->height & 0xFF;
  b[3]  = s->width >> 8;
  b[4]  = s->width & 0xFF;
  b[5]  = 3; /* Components (Y'Cb'Cr) */
  b[6]  = 1; /* Y */
  switch(s->mcu_mode)
  {
  case 0: 
    b[7] = 0x22; 
    break;
  case 1: 
    b[7] = 0x12; 
    break;
  case 2: 
    b[7] = 0x21; 
    break;
  case 3: 
    b[7] = 0x11; 
    break;
  }
  b[8]  = 0x00;
  b[9]  = 2; /* Cb */
  b[10] = 0x11;
  b[11] = 0x01;
  b[12] = 3; /* Cr */
  b[13] = 0x11;
  b[14] = 0x01;
  ssdv_write_marker(s, J_SOF0,  15, b);  /* SOF0 (Baseline DCT) */

  ssdv_write_marker(s, J_DHT,   29, std_dht00); /* DHT (DC Luminance)  */
  ssdv_write_marker(s, J_DHT,  179, std_dht10); /* DHT (AC Luminance)  */
  ssdv_write_marker(s, J_DHT,   29, std_dht01); /* DHT (DC Chrominance */
  ssdv_write_marker(s, J_DHT,  179, std_dht11); /* DHT (AC Chrominance */
  ssdv_write_marker(s, J_SOS,   10, sos);
}

static void ssdv_fill_gap(ssdv_t *s, uint16_t next_mcu)
{
  if(s->mcupart > 0 || s->acpart > 0)
  {
    /* Cleanly end the current MCU part */
    if(s->acpart > 0)
    {
      ssdv_out_jpeg_int(s, 0, 0);
      s->mcupart++;
    }

    /* End the current MCU block */
    for(; s->mcupart < s->ycparts + 2; s->mcupart++)
    {
      if(s->mcupart < s->ycparts) s->component = 0;
      else s->component = s->mcupart - s->ycparts + 1;
      s->acpart = 0; 
      ssdv_out_jpeg_int(s, 0, 0); /* DC */
      s->acpart = 1; 
      ssdv_out_jpeg_int(s, 0, 0); /* AC */
    }

    s->mcu_id++;
  }

  /* Pad out missing MCUs */
  for(; s->mcu_id < next_mcu; s->mcu_id++)
  {
    /* End the current MCU block */
    for(s->mcupart = 0; s->mcupart < s->ycparts + 2; s->mcupart++)
    {
      if(s->mcupart < s->ycparts) s->component = 0;
      else s->component = s->mcupart - s->ycparts + 1;
      s->acpart = 0; 
      ssdv_out_jpeg_int(s, 0, 0); /* DC */
      s->acpart = 1; 
      ssdv_out_jpeg_int(s, 0, 0); /* AC */
    }                         
  }
}

//char ssdv_dec_init(ssdv_t *s)
//{
//	memset(s, 0, sizeof(ssdv_t));
//	sptr=s;
/* The packet data should contain only scan data, no headers */
//	s->state = S_HUFF;
//	s->mode = S_DECODING;

/* Prepare the source JPEG tables */
//	s->sdqt[0] = stblcpy(s, std_dqt0, sizeof(std_dqt0));
//	s->sdqt[1] = stblcpy(s, std_dqt1, sizeof(std_dqt1));
//	s->sdht[0][0] = stblcpy(s, std_dht00, sizeof(std_dht00));
//	s->sdht[0][1] = stblcpy(s, std_dht01, sizeof(std_dht01));
//	s->sdht[1][0] = stblcpy(s, std_dht10, sizeof(std_dht10));
//	s->sdht[1][1] = stblcpy(s, std_dht11, sizeof(std_dht11));

/* Prepare the output JPEG tables */
//	s->ddqt[0] = dtblcpy(s, std_dqt0, sizeof(std_dqt0));
//	s->ddqt[1] = dtblcpy(s, std_dqt1, sizeof(std_dqt1));
//	s->ddht[0][0] = dtblcpy(s, std_dht00, sizeof(std_dht00));
//	s->ddht[0][1] = dtblcpy(s, std_dht01, sizeof(std_dht01));
//	s->ddht[1][0] = dtblcpy(s, std_dht10, sizeof(std_dht10));
//	s->ddht[1][1] = dtblcpy(s, std_dht11, sizeof(std_dht11));

/* Allocate the memory for the JPEG */
/*s->out_len = 64 * 1024;
 	s->out = s->outp = (uint8_t *) malloc(s->out_len);
 	if(!s->out)
 	{
 		fprintf(stderr, "ssdv_dec_init(): Out of memory\n");
 		return(SSDV_ERROR);
 	}*/



//	return(SSDV_OK);
//}

char ssdv_dec_set_buffer(ssdv_t *s, uint8_t *buffer, size_t length)
{
  size_t c = s->outp - s->out;

  s->outp = buffer + c;
  s->out = buffer;
  s->out_len = length - c;

  /* Flush the output bits */
  ssdv_outbits(s, 0, 0);

  return(SSDV_OK);
}

char ssdv_dec_feed(ssdv_t *s, uint8_t *packet)
{
  int i = 0, r;
  uint8_t b;
  uint16_t packet_id;

  /* Read the packet header */
  packet_id            = (packet[7] << 8) | packet[8];
  s->packet_mcu_offset = packet[12];
  s->packet_mcu_id     = (packet[13] << 8) | packet[14];
  hptr->packetid = packet_id;
  hptr->imageid = packet[6];
  hptr->mcuid= s->mcu_id;
  if(s->packet_mcu_id != 0xFFFF) s->reset_mcu = s->packet_mcu_id;

  /* If this is the first packet, write the JPEG headers */
  if(s->packet_id == 0)
  {
    const char *factor;
    char callsign[7];

    /* Read the fixed headers from the packet */
    s->callsign  = (packet[2] << 24) | (packet[3] << 16) | (packet[4] << 8) | packet[5];
    s->image_id  = packet[6];
    s->width     = packet[9] << 4;
    s->height    = packet[10] << 4;
    s->mcu_count = packet[9] * packet[10];
    s->mcu_mode  = packet[11] & 0x03;

    switch(s->mcu_mode)
    {
    case 0: 
      factor = "2x2"; 
      s->ycparts = 4; 
      break;
    case 1: 
      factor = "1x2"; 
      s->ycparts = 2; 
      s->mcu_count *= 2; 
      break;
    case 2: 
      factor = "2x1"; 
      s->ycparts = 2; 
      s->mcu_count *= 2; 
      break;
    case 3: 
      factor = "1x1"; 
      s->ycparts = 1; 
      s->mcu_count *= 4; 
      break;
    }

    /* Display information about the image */
    fprintf(stderr, "Callsign: %s\n", decode_callsign(callsign, s->callsign));
    fprintf(stderr, "Image ID: %02X\n", s->image_id);
    fprintf(stderr, "Resolution: %ix%i\n", s->width, s->height);
    fprintf(stderr, "MCU blocks: %i\n", s->mcu_count);
    fprintf(stderr, "Sampling factor: %s\n", factor);

    /* Output JPEG headers and enable byte stuffing */
    ssdv_out_headers(s);
    s->out_stuff = 1;

    hptr->lostpacket= 0;


    hptr->width = s->width;
    hptr->height=s->height;

    hptr->mcucount = s->mcu_count;
    decode_callsign(hptr->callsign, s->callsign);

  }
  //lostpacket=lostpacket+1;
  /* Is this not the packet we expected? */
  if(packet_id != s->packet_id)
  {
    /* One or more packets have been lost! */
    fprintf(stderr, "Gap detected between packets %i and %i\n", s->packet_id - 1, packet_id);
    hptr->lostpacket = hptr->lostpacket+ (packet_id - s->packet_id);
    /* If this packet has no new MCU, ignore */
    if(s->packet_mcu_offset == 0xFF) return(SSDV_FEED_ME);

    /* Fill the gap left by the missing packet */
    ssdv_fill_gap(s, s->packet_mcu_id);

    /* Clear the workbits */
    s->workbits = s->worklen = 0;

    /* Skip the bytes of the lost MCU */
    i = s->packet_mcu_offset;

    /* Reset the JPEG decoder state */
    s->state = S_HUFF;
    s->component = 0;
    s->mcupart = 0;
    s->acpart = 0;
    s->accrle = 0;

    s->packet_id = packet_id;
  }

  /* Feed the JPEG data into the processor */
  for(; i < SSDV_PKT_SIZE_PAYLOAD; i++)
  {
    b = packet[SSDV_PKT_SIZE_HEADER + i];

    /* Add the new byte to the work area */
    s->workbits = (s->workbits << 8) | b;
    s->worklen += 8;

    /* Process the new data until more needed, or an error occurs */
    while((r = ssdv_process(s)) == SSDV_OK);
    hptr->mcuid=s->mcu_id;
    if(r == SSDV_BUFFER_FULL)
    {
      /* Realloc memory */
    }
    else if(r == SSDV_EOI)
    {
      /* All done! */

      return(SSDV_OK);
    }
    else if(r != SSDV_FEED_ME)
    {
      /* An error occured */
      fprintf(stderr, "ssdv_process() failed: %i\n", r);
      return(SSDV_ERROR);
    }
  }

  /* The next packet to expect... */
  s->packet_id++;

  return(SSDV_FEED_ME);
}

char ssdv_dec_get_jpeg(ssdv_t *s, uint8_t **jpeg, size_t *length)
{
  /* Is the image complete? */
  if(s->mcu_id < s->mcu_count) ssdv_fill_gap(s, s->mcu_count);

  /* Sync, and final EOI header and return */
  ssdv_outbits_sync(s);
  s->out_stuff = 0;
  ssdv_write_marker(s, J_EOI, 0, 0);

  *jpeg = s->out;
  *length = (size_t) (s->outp - s->out);

  return(SSDV_OK);
}



char ssdv_dec_is_packet(uint8_t *packet, int *errors)
{
  uint8_t pkt[SSDV_PKT_SIZE];
  ssdv_packet_info_t p;
  uint32_t x;
  int i;

  /* Testing is destructive, work on a copy */
  memcpy(pkt, packet, SSDV_PKT_SIZE);
  pkt[0] = 0x55;
  pkt[1] = 0x66;

  /* Run the reed-solomon decoder */
  //i = decode_rs_8(&pkt[1], 0, 0, 0); //Commented out so that it compiles.
  if(i < 0) return(-1); /* Reed-solomon decoder failed */
  hptr->errorcor=i;
  if(errors) *errors = i;

  /* Sanity checks */
  if(pkt[1] != 0x66) return(-1);

  ssdv_dec_header(&p, pkt);
  if(p.width == 0 || p.height == 0) return(-1);
  if(p.mcu_id != 0xFFFF)
  {
    if(p.mcu_id >= p.mcu_count) return(-1);
    if(p.mcu_offset >= SSDV_PKT_SIZE_PAYLOAD) return(-1);
  }

  /* Test the checksum */
  x = crc32(&pkt[1], SSDV_PKT_SIZE_CRCDATA);

  i = 1 + SSDV_PKT_SIZE_CRCDATA;
  if(pkt[i++] != ((x >> 24) & 0xFF)) return(-1);
  if(pkt[i++] != ((x >> 16) & 0xFF)) return(-1);
  if(pkt[i++] != ((x >> 8) & 0xFF)) return(-1);
  if(pkt[i++] != (x & 0xFF)) return(-1);

  /* Appears to be a valid packet! Copy it back */
  memcpy(packet, pkt, SSDV_PKT_SIZE);

  return(0);
}
void ssdv_dec_header(ssdv_packet_info_t *info, uint8_t *packet)
{
  info->callsign  = (packet[2] << 24) | (packet[3] << 16) | (packet[4] << 8) | packet[5];
  info->image_id   = packet[6];
  info->packet_id  = (packet[7] << 8) | packet[8];
  info->width      = packet[9] << 4;
  info->height     = packet[10] << 4;
  info->mcu_mode   = packet[11] & 0x03;
  info->mcu_offset = packet[12];
  info->mcu_id     = (packet[13] << 8) | packet[14];
  info->mcu_count  = packet[9] * packet[10];
  if(info->mcu_mode == 1 || info->mcu_mode == 2) info->mcu_count *= 2;
  else if(info->mcu_mode == 3) info->mcu_count *= 4;
}
int encodeImage(char imgid,char Callsign[])		
{

  char callsign[7];
  uint8_t pkt[SSDV_PKT_SIZE], img[32];	
  uint8_t image_id = 0;
  strncpy(callsign, Callsign, 6);
  image_id=imgid;
 // static uint8_t img_id = 0;
  static ssdv_t ssdv;
  int c,i=0;

  /* Encode */

  ssdv_enc_init(&ssdv, callsign, image_id);
  ssdv_enc_set_buffer(&ssdv, pkt);

  while(1)
  {
    starttimer();
    while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME && gettimerstatus())
    //  while(c = ssdv_enc_get_packet(&ssdv) == SSDV_FEED_ME)
    {
      starttimer();
      size_t r = getPicture(&img[0]);

     // if(r == 1)
     // {
     //   Serial.println("Premature end of file");
     //   break;
     // }
      ssdv_enc_feed(&ssdv, img, 32);
    }
    
    stoptimer();
    if(c == SSDV_EOI)
    {
    //  Serial.println("ssdv_enc_get_packet said EOI");
      break;
    }
    else if(c != SSDV_OK)
    {
       Serial.println("ssdv_enc_get_packet failed");
      return(-1);
    }

   // fwrite(pkt, 1, SSDV_PKT_SIZE, fout);//sent to transmitter packet
  // Serial.print("Send Packet to TX ");
   //Serial.println(i);
   if(i==0){
    // sendTelemetry();
     
     sendSSDVpic(pkt);
   }else
 //  sendTelemetry();
   sendSSDVpic(pkt);
   
    i++;
    /*Send Telemetry every 10 packets*/
    if (i % 5 == 0){
      sendTelemetry();
    }
  }
 //  fprintf(stderr, "Wrote packets %i\n", i);
   // Serial.print("Wrote packets ");
  //  Serial.println(c);
     //sendSSDVpic(pkt);
    return(c);
}






