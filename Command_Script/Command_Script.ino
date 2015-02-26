void sendCommand(String command)
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
	sendCommand("C,A,W,500");
	sendCommand("C,G,L");
	Serial.println("Finished");

}



void loop()
{
}
