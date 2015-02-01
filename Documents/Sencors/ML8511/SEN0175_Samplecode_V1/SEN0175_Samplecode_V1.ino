/***************************************************
 * UV Sensor v1.0-ML8511
 * <....>
 ***************************************************
 * This example reads UV intensity from UV Sensor v1.0-ML8511.
 * 
 * Created 2014-9-23
 * By Phoebe <phoebe.wang@dfrobot.com>
 * Modified 2014-9-23
 * By Phoebe phoebe.wang@dfrobot.com>
 * 
 * GNU Lesser General Public License.
 * See <http://www.gnu.org/licenses/> for details.
 * All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 * 1.Connect ML8511 UV Sensor to Arduino A0
 * <http://wiki.dfrobot.com.cn/index.php/%E6%96%87%E4%BB%B6:SEN0175_Diagram.png>
 * 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

const int ReadUVintensityPin = A0; //Output from the sensor

void setup()
{
  pinMode(ReadUVintensityPin, INPUT);
  Serial.begin(9600); //open serial port, set the baud rate to 9600 bps
  Serial.println("Starting up...");
}

void loop()
{
  int uvLevel = averageAnalogRead(ReadUVintensityPin);

  float outputVoltage = 5.0 * uvLevel/1024;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

  Serial.print("UVAnalogOutput: ");
  Serial.print(uvLevel);

  Serial.print(" OutputVoltage: ");
  Serial.print(outputVoltage);

  Serial.print(" UV Intensity: ");
  Serial.print(uvIntensity);
  Serial.print(" mW/cm^2");

  Serial.println(); 
  delay(100);
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  

}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

