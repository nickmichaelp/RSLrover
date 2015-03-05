 
{
	for(int i = 0; i < 100; i++)
	{
		Serial1.println(command);
	}
	Serial.println("Done");
}

void setup()
{
	Serial.begin(2400);
        Serial1.begin(2400);
	sendCommand("C,G,N");
        sendCommand("C,A,V,-500");
        sendCommand("C,A,V,100");
	Serial.println("Finished");

}



void loop()
{
}
