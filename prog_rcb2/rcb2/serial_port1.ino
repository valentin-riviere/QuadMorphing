/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*

To check :

If read_buffer is the last stream received when reading uart


*/





#define NB_UINT32_BYTE  4
#define NB_INT32_BYTE   4
#define NB_UINT8_BYTE  1
#define NB_INT8_BYTE   1
#define NB_UINT16_BYTE  2
#define NB_INT16_BYTE   2

#define TIME_PER_BYTE_BR115200 87  // time in us needed to transmit one byte with a baudrate of 115200/************ START STREAM *************/

uint8_t SendStartStream_RS232(void)
{
  return(Serial1.write("START")); 
}

uint8_t StartAsked(void)
{
  char *StartMessage = "START";
  uint8_t StartAsked = 1;
  uint8_t ByteRead;
  uint8_t i;
  
  if (Serial1.available()>=strlen(StartMessage))
  {
    // verifiy that the received stream is well the start stream
    for (i=0; i<strlen(StartMessage); i++)
    {
      ByteRead = Serial1.read();
		#if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
			Serial.print("ByteRead: ");
			Serial.println(ByteRead);
		#endif
      if ((uint8_t)StartMessage[i] == ByteRead)
      {
        // do nothing
      }
      else
      {
        StartAsked = 0;
        break;
      }
    }
  }
  else
  {
    StartAsked = 0;
  }
  
  if (StartAsked == 1)
  {
    Serial.println("START stream received!");
  }
  else
  {
    // nothing
  }
  
  return(StartAsked);
}

/***************** SEND ****************/

// send int32
uint8_t HeaderSendnInt32_RS232(int32_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
    Serial1.write(&Header,1);
    return(Serial1.write((const uint8_t*)ToSend, NbToSend*NB_INT32_BYTE));
}

uint8_t SendnInt32_RS232(int32_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend,NbToSend*NB_INT32_BYTE);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_INT32_BYTE);
    Serial.println(" Int32 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend,NbToSend*NB_INT32_BYTE)); 
  #endif
}

// send Uint32
uint8_t HeaderSendnUInt32_RS232(uint32_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
    Serial1.write(&Header,1);
    return(Serial1.write((const uint8_t*)ToSend, NbToSend*NB_UINT32_BYTE));
}

uint8_t SendnUInt32_RS232(uint32_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend,NbToSend*NB_UINT32_BYTE);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_UINT32_BYTE);
    Serial.println(" UInt32 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend,NbToSend*NB_UINT32_BYTE)); 
  #endif
}

// send int16
uint8_t HeaderSendnInt16_RS232(int16_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
  Serial1.write(Header);
  return(Serial1.write((const uint8_t*)ToSend, NbToSend*NB_INT16_BYTE));
}

uint8_t SendnInt16_RS232(int16_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend,NbToSend*NB_INT16_BYTE);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_INT16_BYTE);
    Serial.println(" Int16 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend,NbToSend*NB_INT16_BYTE)); 
  #endif
}

// send UInt16
uint8_t HeaderSendnUInt16_RS232(uint16_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
    Serial1.write(&Header,1);
    return(Serial1.write((const uint8_t*)ToSend, NbToSend*NB_UINT16_BYTE));
}

uint8_t SendnUInt16_RS232(uint16_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend,NbToSend*NB_UINT16_BYTE);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_UINT16_BYTE);
    Serial.println(" UInt16 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend,NbToSend*NB_UINT16_BYTE)); 
  #endif
}

// send int8
uint8_t HeaderSendnInt8_RS232(int8_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
    Serial1.write(&Header,1);
    return(Serial1.write((const uint8_t*)ToSend, NbToSend));
}

uint8_t SendnInt8_RS232(int8_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend,NbToSend);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_INT8_BYTE);
    Serial.println(" Int8 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend, NbToSend)); 
  #endif
}

// send Uint8
uint8_t HeaderSendnUInt8_RS232(uint8_t *ToSend, uint8_t NbToSend, uint8_t Header)
{
    Serial1.write(&Header,1);
    return(Serial1.write((const uint8_t*)ToSend, NbToSend));
}

uint8_t SendnUInt8_RS232(uint8_t *ToSend, uint8_t NbToSend)
{
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    int8_t NbSent;
    int8_t i;
    
    NbSent = Serial1.write((const uint8_t*)ToSend, NbToSend);
    Serial.print("SERIAL 1: ");
    Serial.print(NbSent/NB_UINT8_BYTE);
    Serial.println(" UInt8 sent:");
    Serial.print("   ");
    for(i=0; i<NbSent/2-1; i++)
    {
      Serial.print(ToSend[i]);
      Serial.print(" ");
    }
    if (i!= 0)
    {
      Serial.println(ToSend[NbSent/2-1]);
    }
    else
    {
    }
    
    return(NbSent);  
  #else
    return(Serial1.write((const uint8_t*)ToSend, NbToSend)); 
  #endif
}

/***************** RECEIVE ****************/

// wait int32
void WaitnInt32_RS232(int32_t *ToRead, uint8_t NbToWait)
{
    // wait to read n int32 (limited to 256 int32) 
       // return the number of Int32 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait*NB_INT32_BYTE))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = ((int32_t)Serial1.read() | (int32_t)Serial1.read()<<8 | (int32_t)Serial1.read()<<16 | (int32_t)Serial1.read()<<24);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnInt32_RS232(int32_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n int32 (limited to 256 int32) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of Int32 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read;
  
  Header_read = Serial1.read();
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.print("Header: ");
    Serial.println(Header_read);
  #else
   if (Serial1.available() == 0)
   {
     delayMicroseconds(TIME_PER_BYTE_BR115200);
   }
  #endif
  while(Serial1.available() && Header_read!=Header)
  {
    Header_read = Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Header: ");
      Serial.println(Header_read);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
  }
  delayMicroseconds(TIME_PER_BYTE_BR115200*NB_INT32_BYTE);
  while((Serial1.available() >= NB_INT32_BYTE) && NbRead < NbToRead) 
  {
    ToRead[NbRead] = ((int32_t)Serial1.read() | (int32_t)Serial1.read()<<8 | (int32_t)Serial1.read()<<16 | (int32_t)Serial1.read()<<24);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.println(ToRead[NbRead]);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200*NB_INT32_BYTE);
      }
    #endif
    NbRead ++;
  }
  return(NbRead);
}

int8_t ReadnInt32_RS232(int32_t *ToRead, uint8_t NbToRead)
{
      // try to read n int32 (limited to 256 int32)
       // return the number of Int32 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= NB_INT32_BYTE) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = ((int32_t)Serial1.read() | (int32_t)Serial1.read()<<8 | (int32_t)Serial1.read()<<16 | (int32_t)Serial1.read()<<24);
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" int32 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}

// Uint32
void WaitnUInt32_RS232(uint32_t *ToRead, uint8_t NbToWait)
{
    // wait to read n UUInt16 (limited to 256 UUInt16) 
       // return the number of UUInt16 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait*NB_UINT32_BYTE))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = ((uint32_t)Serial1.read() | (uint32_t)Serial1.read()<<8| (uint32_t)Serial1.read()<<16 | (uint32_t)Serial1.read()<<24);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnUInt32_RS232(uint32_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n uint32 (limited to 256 int32) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of Int32 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read;
  
  Header_read = Serial1.read();
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.print("Header: ");
    Serial.println(Header_read);
  #else
   if (Serial1.available() == 0)
   {
     delayMicroseconds(TIME_PER_BYTE_BR115200);
   }
  #endif
  while(Serial1.available() && Header_read!=Header)
  {
    Header_read = Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Header: ");
      Serial.println(Header_read);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
  }
  delayMicroseconds(TIME_PER_BYTE_BR115200*NB_UINT32_BYTE);
  while((Serial1.available() >= NB_INT32_BYTE) && NbRead < NbToRead) 
  {
    ToRead[NbRead] = ((uint32_t)Serial1.read() | (uint32_t)Serial1.read()<<8| (uint32_t)Serial1.read()<<16 | (uint32_t)Serial1.read()<<24);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.println(ToRead[NbRead]);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200*NB_UINT32_BYTE);
      }
    #endif
    NbRead ++;
  }
  return(NbRead);
}

int8_t ReadnUInt32_RS232(uint32_t *ToRead, uint8_t NbToRead)
{
      // try to read n uint32 (limited to 256 UUInt16)
       // return the number of uInt32 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= NB_UINT32_BYTE) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = ((uint32_t)Serial1.read() | (uint32_t)Serial1.read()<<8) |( uint32_t)Serial1.read()<<16 | (uint32_t)Serial1.read()<<24;
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" uint32 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}

// Int16
void WaitnInt16_RS232(int16_t *ToRead, uint8_t NbToWait)
{
    // wait to read n Int16 (limited to 256 Int16) 
       // return the number of Int16 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait*NB_INT16_BYTE))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = ((int16_t)Serial1.read() | (int16_t)Serial1.read()<<8);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnInt16_RS232(int16_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n Int16 (limited to 256 Int16) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of Int16 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read = 0;
  
  // Header_read = Serial1.read();
  // #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    // Serial.print("Header 1: ");
    // Serial.println(Header_read);
  // #endif
  
   // if (Serial1.available() == 0)
   // {
     // delayMicroseconds(TIME_PER_BYTE_BR115200);
   // }

  // while(Serial1.available() < 2*(NbToRead+1) && Header_read!=Header && Serial1.available() > 0)
  // {
    // Header_read = Serial1.read();
	
    // #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      // Serial.print("Header 2: ");
      // Serial.println(Header_read);
    // #endif
  // }
  // if (Header_read == Header)
  // {
		// delayMicroseconds(TIME_PER_BYTE_BR115200*NB_INT16_BYTE);
		// while((Serial1.available() >= NB_INT16_BYTE) && NbRead < NbToRead) 
		// {
			// ToRead[NbRead] = ((int16_t)Serial1.read() | (int16_t)Serial1.read()<<8);
			// #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
				// Serial.println("Read bytes: ");
				// Serial.println(ToRead[NbRead]);
			// #endif

			// NbRead++;
		// }
  // }
  
	boolean header_rcv = false;

	while((Serial1.available() >= NB_INT16_BYTE) && NbRead < NbToRead) 
	{
		delayMicroseconds(TIME_PER_BYTE_BR115200);
		
cli();
		ByteRead = Serial1.read();
sei();
		#if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
			Serial.println("Read bytes: ");
			Serial.println(ByteRead);
		#endif
		if (ByteRead == Header)
		{
			header_rcv = true;
			NbRead == 0;
		}
		else if (header_rcv)
		{
			delayMicroseconds(TIME_PER_BYTE_BR115200);
cli();
			ToRead[NbRead] = ((int16_t)ByteRead | (int16_t)Serial1.read()<<8);
sei();

			#if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
				Serial.println("Read int16_t: ");
				Serial.println(ToRead[NbRead]);
			#endif
			
			NbRead++;
		}
	}

  return(NbRead);
}

int8_t ReadnInt16_RS232(int16_t *ToRead, uint8_t NbToRead)
{
      // try to read n Int16 (limited to 256 Int16)
       // return the number of Int16 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= NB_INT16_BYTE) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = ((int16_t)Serial1.read() | (int16_t)Serial1.read()<<8);
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" Int16 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}

// UInt16
void WaitnUInt16_RS232(uint16_t *ToRead, uint8_t NbToWait)
{
    // wait to read n UInt16 (limited to 256 UInt16) 
       // return the number of UInt16 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait*NB_UINT16_BYTE))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = ((uint16_t)Serial1.read() | (uint16_t)Serial1.read()<<8);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnUInt16_RS232(uint16_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n UInt16 (limited to 256 UInt16) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of UInt16 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read;
  
  Header_read = Serial1.read();
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.print("Header: ");
    Serial.println(Header_read);
  #else
   if (Serial1.available() == 0)
   {
     delayMicroseconds(TIME_PER_BYTE_BR115200);
   }
  #endif
  while(Serial1.available() && Header_read!=Header)
  {
    Header_read = Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Header: ");
      Serial.println(Header_read);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
  }
  delayMicroseconds(TIME_PER_BYTE_BR115200*NB_UINT16_BYTE);
  while((Serial1.available() >= NB_UINT16_BYTE) && NbRead < NbToRead) 
  {
    ToRead[NbRead] = ((uint16_t)Serial1.read() | (uint16_t)Serial1.read()<<8);
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.println(ToRead[NbRead]);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200*NB_UINT16_BYTE);
      }
    #endif
    NbRead ++;
  }
  return(NbRead);
}

int8_t ReadnUInt16_RS232(uint16_t *ToRead, uint8_t NbToRead)
{
      // try to read n UInt16 (limited to 256 UInt16)
       // return the number of UInt16 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= NB_UINT16_BYTE) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = ((uint16_t)Serial1.read() | (uint16_t)Serial1.read()<<8);
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" UInt16 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}

// Int8
void WaitnInt8_RS232(int8_t *ToRead, uint8_t NbToWait)
{
    // wait to read n Int8 (limited to 256 Int8) 
       // return the number of Int8 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = (int8_t)Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnInt8_RS232(int8_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n Int8 (limited to 256 Int8) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of Int8 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read;
  
  Header_read = Serial1.read();
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.print("Header: ");
    Serial.println(Header_read);
  #else
   if (Serial1.available() == 0)
   {
     delayMicroseconds(TIME_PER_BYTE_BR115200);
   }
  #endif
  while(Serial1.available() && Header_read!=Header)
  {
    Header_read = Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Header: ");
      Serial.println(Header_read);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
  }
  delayMicroseconds(TIME_PER_BYTE_BR115200);
  while((Serial1.available() >= 1) && NbRead < NbToRead) 
  {
    ToRead[NbRead] = (int8_t)Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.println(ToRead[NbRead]);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
    NbRead ++;
  }
  return(NbRead);
}

int8_t ReadnInt8_RS232(int8_t *ToRead, uint8_t NbToRead)
{
      // try to read n Int8 (limited to 256 Int8)
       // return the number of Int8 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= 1) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = (int8_t)Serial1.read();
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" Int8 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}

// UInt8
void WaitnUInt8_RS232(uint8_t *ToRead, uint8_t NbToWait)
{
    // wait to read n UInt8 (limited to 256 UInt8) 
       // return the number of UInt8 read
  uint8_t i;
  
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.println("Begin Wait...");
  #endif
  
  while(Serial1.available()<(NbToWait))
  {
    delayMicroseconds(1);
  }
  for (i=0;i<NbToWait;i++)
  {
    ToRead[i] = (uint8_t)Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Received[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(ToRead[i]);
    #endif
  }
}

int8_t HeaderReadnUInt8_RS232(uint8_t *ToRead, uint8_t NbToRead, uint8_t Header)
{
    // try to read n UInt8 (limited to 256 UInt8) with a byte as header (first byte is not data, but a header which indicates the begin of the stream)
       // return the number of UInt8 read
  uint8_t ByteRead;
  uint8_t NbRead = 0; 
  uint8_t Header_read;
  
  Header_read = Serial1.read();
  #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
    Serial.print("Header: ");
    Serial.println(Header_read);
  #else
   if (Serial1.available() == 0)
   {
     delayMicroseconds(TIME_PER_BYTE_BR115200);
   }
  #endif
  while(Serial1.available() && Header_read!=Header)
  {
    Header_read = Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("Header: ");
      Serial.println(Header_read);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
  }
  delayMicroseconds(TIME_PER_BYTE_BR115200);
  while((Serial1.available() >= 1) && NbRead < NbToRead) 
  {
    ToRead[NbRead] = (uint8_t)Serial1.read();
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.println(ToRead[NbRead]);
    #else
      if (Serial1.available() == 0)
      {
        delayMicroseconds(TIME_PER_BYTE_BR115200);
      }
    #endif
    NbRead ++;
  }
  return(NbRead);
}

int8_t ReadnUInt8_RS232(uint8_t *ToRead, uint8_t NbToRead)
{
      // try to read n UInt8 (limited to 256 UInt8)
       // return the number of UInt8 read
    uint8_t ByteRead;
    uint8_t NbRead = 0;  
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      uint8_t i;
    #endif
    
    while((Serial1.available() >= 1) && (NbRead < NbToRead)) 
    {
      ToRead[NbRead] = (uint8_t)Serial1.read();
      NbRead ++;
    }
    #if defined(ECHO_PRINTF) || defined(ECHO_ONLY_GUM)
      Serial.print("SERIAL 1: ");
      Serial.print(NbRead);
      Serial.println(" UInt8 read:");
      Serial.print("   ");
      for(i=0; i<NbRead-1; i++)
      {
        Serial.print(ToRead[i]);
        Serial.print(" ");
      }
      if (i!= 0)
      {
        Serial.println(ToRead[NbRead-1]);
      }
    #endif
    
    return(NbRead);
}


