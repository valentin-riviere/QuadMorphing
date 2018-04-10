/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
bool led_state;
uint8_t cpt_led = 0;

void led_init()
{
	pinMode(PIN_LED,OUTPUT);
	digitalWrite(PIN_LED,LOW);
	led_state = LOW;
}

void led_turn_on()
{
	digitalWrite(PIN_LED,HIGH);
	led_state = HIGH;
}

void led_turn_off()
{
	digitalWrite(PIN_LED,LOW);
	led_state = LOW;
}

void led_blink_1s()
{
	digitalWrite(PIN_LED,HIGH);
	delay(1000);
	digitalWrite(PIN_LED,LOW);
}

void led_sw_state()	// Blink LED for cpt_led < 128
{
	cpt_led++;
	
	if ((cpt_led & 0x10) == 0)
	{
		if (led_state == LOW)
			led_turn_on();
		else
			led_turn_off();
	}
}