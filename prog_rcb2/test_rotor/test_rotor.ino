#include <stdio.h>
#include <ctype.h>

using namespace std;

int val_read = 0;

void setup()
{
	Serial.begin(921600);
	delay(500);

	pinMode(5,OUTPUT); // motor 2
	pinMode(6,OUTPUT); // motor 1
	
	analogWriteResolution(11);
}

void loop()
{
	String inString = "";

	analogWrite(5,(float) val_read/2000*2047);
	analogWrite(6,(float) val_read/2000*2047);
	
	if (Serial.available() > 0)
	{
		while (Serial.available() > 0)
		{
			int inChar = Serial.read();
			
			inString += (char) inChar;
		}
		val_read = inString.toInt();
		inString = "";
	}

	Serial.println(val_read);
	delay(100);
}