void sendCommand(String command)
{
	Serial.println(command);
	for(int i = 0; i < 100; i++)
	{
		Serial1.println(command);
	}
	Serial.println("Done");
}
void setup()
{
	Serial.begin(9600);
	Serial1.begin(9600);
	sendCommand("C,A,W,500");
	sendCommand("C,A,V,-750");
	sendCommand("C,G,L");
	Serial.println("Finished");
}

void loop()
{
}
