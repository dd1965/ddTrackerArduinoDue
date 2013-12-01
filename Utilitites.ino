

ushort rtty_CRC16_checksum (byte rxByte[],int length) 
{
  //$$PSB,0001,000000,0.0,0.0,0,0,0,0,107,26,7656*16B3     //Test string   CRC on everything between $$ and * last 4 byte is rx crc
  ushort crc=0xFFFF;
  //String ln= "Length " + String(length);
  // Serial.print(ln);
  byte c;    
  // Calculate checksum ignoring the first two $s
  for (int i = 0; i < length-1; i++)//46
  {
    c = rxByte[i];
    //  Serial.print(c,HEX);
    // Serial.print(" ");
    ushort cr = (ushort)(c << 8);
    crc = (ushort)(crc ^ (cr));
    for (int x = 0; x < 8; x++)
    {
      if ((crc & 0x8000) == 0x8000)
        crc = (ushort)((crc << 1) ^ 0x1021);
      else
        crc = (ushort) (crc << 1);
    }               
  }

  return crc;
}
void encodeGold(int Telemetry)
{
  if (Telemetry == 0)
  {
    //write preamble and correlation tag here
    encodeArray[0] = 0x3E;
    encodeArray[1] = 0x3E;
    encodeArray[2] = 0x3E;
    encodeArray[3] = 0x3E;
    // Tag_0 is 0xB7 4D B7 DF 8A 53 2F 3E
    //  
    // appropriate for RS(255,239) FEC coding
    //
    encodeArray[4] = 0x3E;
    encodeArray[5] = 0x2F;
    encodeArray[6] = 0x53;
    encodeArray[7] = 0x8A;
    encodeArray[8] = 0xDF;
    encodeArray[9] = 0xB7;
    encodeArray[10] = 0x4D;
    encodeArray[11] = 0xB7;
    // encodeArray[12] = 0x24;
  }
  else
  {//write preamble and correlation tag here
    encodeArray[0] = 0x3E;
    encodeArray[1] = 0x3E;
    encodeArray[2] = 0x3E;
    encodeArray[3] = 0x3E;
    // 0x6E 26 0B1A C5 83 5F AE
    // appropriate for RS(255,239) FEC coding
    /* encodeArray[4] = 0xAE;
     encodeArray[5] = 0x5F;
     encodeArray[6] = 0x83;
     encodeArray[7] = 0xC5;
     encodeArray[8] = 0x1A;
     encodeArray[9] = 0x0B;
     encodeArray[10] = 0x26;
     encodeArray[11] = 0x6E;*/

    //
    //0x3A DB 0C 13 DE AE 28 36
    encodeArray[4] = 0x36;
    encodeArray[5] = 0x28;
    encodeArray[6] = 0xAE;
    encodeArray[7] = 0xDE;
    encodeArray[8] = 0x13;
    encodeArray[9] = 0x0C;
    encodeArray[10] = 0xDB;
    encodeArray[11] = 0x3A;
    encodeArray[12] = 0x24;
  }
}
// return //encodeArray;
void shuffle(byte *buf)
{
  Serial.println();                               
  for (int i = 0; i < 256; i++)
  {

    cbyte=buf[i];
    for (int bi = 0; bi < 8; bi++)
    {
      symbols[bi + (8 * i)]=cbyte&0x80;
      if(symbols[bi + (8 * i)]==0x80) 
      {
        symbols[bi + (8 * i)]=1;
      }
      cbyte=cbyte<<1;
    //  Serial.print(symbols[bi + (8 * i)]);
    }   
   // Serial.println();          
  }


  interleaver();
  cbyte=0;
  for (int i = 0; i < 256; i++)
  {

    for (int bi = 0; bi < 8; bi++)
    {

      bitin = symbols_interleaved[bi + (8 * i)];
      cbyte = cbyte << 1;
      cbyte = cbyte | bitin;

      //Console.WriteLine(cbyte);
    }
    // Console.WriteLine(cbyte);
    buf[i] = cbyte;
    cbyte = 0;
  }
  //  deshuffle(decbuf);

}


void interleaver ()//Input symbols
{

  int i, j, k, l, P;

  P = 0;
  while (P < numbits)
  {
    for (k = 0; k <= 2047; k++)                        // bits reverse, ex: 0010 1110 --> 0111 0100
    {
      i = k;
      j = 0;
      for (l = 10; l >= 0; l--)                      // hard work is done here...
      {
        j = j | (i & 0x01) << l;
        i = i >> 1;
      }
      if (j < numbits)
        symbols_interleaved[j] = symbols[P++];        // range in interleaved table
    }                                             // end of while, interleaved table is full
  }

}


