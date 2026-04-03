//define connections
#define IRQ 2 //interrupt at data ready
#define SS 3  //sample start

//define measurement unit (uncomment the chosen unit - default: MM)
//#define CM //output in cm
#define MM      //output in mm
//#define INCH    //output in inch

//is calibration required?
#define USER_CALIBRATION false    //the user wants to recalibrate the sensor
#define FACTORY_CALIBRATION true  //default calibration data restoring
//note: if both calibration types are set to false, the user calibration data will be loaded
#define CALIBRATION_DISTANCE 0.15 //actual distance on which the sensor is calibrated (in meters)

/*------------------------------------------------------------------------*/

//include headers
#include <Wire.h> //library for I2C communication
#include <math.h> //library for mathematical functions

/*------------------------------------------------------------------------*/

//define I2C parameters
#define i2c_address 0x57  //i2c address of the pmod
#define i2c_EEPROM 0x50   //i2cc address of the EEPROM
#define i2c_frequency 100 //communication frequency in KHz

//define EEPROM addresses
#define EEPROM_factory 0x20 //factory calibration data starting address
#define EEPROM_user 0x10    //user calibration data starting address

/*------------------------------------------------------------------------*/

//factory calibration values
unsigned char ctrl_registers[] = {0x10, 0x11, 0x13, 0x60, 0x18, 0x19, 0x90, 0x91};                               //used control register addresses
unsigned char cali_registers[] = {0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30}; //used calibrationregister addresses
unsigned char ctrl_values[] = {0x04, 0x6E, 0x71, 0x01, 0x22, 0x22, 0x0F, 0xFF};                                  //default control register values

/*------------------------------------------------------------------------*/

//function prototypes
void ToF_begin(void);                                                                     //initializes the sensor
void ToF_start(void);                                                                     //starts a measurement
float ToF_getMeasurement(void);                                                           //displays the measured distance on the serial monitor
void display_unit(void);                                                                  //display the measurement unit on the serial monitor
void ToF_calibrate(void);                                                                 //calibrate the sensor
void ToF_calibrate_magnitude(void);                                                       //perfomr magnitude calibration
void ToF_calibrate_crosstalk(unsigned int avg_nr = 100);                                  //performe crosstalk calibration
void ToF_calibrate_distance(unsigned int reference_distance, unsigned int avg_nr = 100);  //perform distance calibration
unsigned char read_reg(unsigned char reg);                                                //read a register
void write_reg(unsigned char reg, unsigned char val);                                     //write a register
void double2bytes(double nr, unsigned char *exp, unsigned char *msb, unsigned char *lsb); //convert a double to bytes
double bytes2double(unsigned char exp, unsigned char msb, unsigned char lsb);             //convert 3 bytes to double
void debug(void);                                                                         //display all registers
void EEPROM_read(unsigned char address);                                                  //read values from EEPROM
void EEPROM_write(unsigned char address);                                                 //write data to EEPROM

/*------------------------------------------------------------------------*/

// Calibration offset to correct the 10 cm error
#define CALIBRATION_OFFSET 10.0 // in cm

void setup()
{
  Serial.begin(9600);            //initialize serial communication
  Serial.println("Starting..."); //display a message

  pinMode(SS, OUTPUT);        //set pin as output
  digitalWrite(SS, HIGH);     //don't start a measurement yet
  pinMode(IRQ, INPUT_PULLUP); //set pin as input
  delay(1000);                //allow power up

  Serial.println("Initializing sensor...");
  ToF_begin(); //initialize the pmod
  Serial.println("Initialization complete.");
  delay(1000); //wait one second (make the message readable)
}

/*------------------------------------------------------------------------*/

void loop()
{
  Serial.println();                      //leave a line out
  Serial.println("Measurement started"); //display a message
  float distance = ToF_getMeasurement(); //get distance
  Serial.print("Corrected distance: ");  //output a message
  Serial.print(distance);                //output the result
  display_unit();                        //display measurement unit
  delay(1000);                           //wait 1s until the next measurement
}

/*------------------------------------------------------------------------*/

/*
  initializes the sensor in default mode
  arguments: nothing
  returns: nothing
*/
void ToF_begin(void)
{
  Wire.begin();                        // initialization of I2C bus
  Wire.setClock(i2c_frequency * 1000); // set communication frequency

  write_reg(0x01, 0x00); //enable the chip
  write_reg(0xB0, 0xD1); //soft clear - reset all registers, stop conversions

  for (int i = 0; i < 8; i++) //load the first 8 register from the list to initialize the sensor
  {
    write_reg(ctrl_registers[i], ctrl_values[i]); //load the register
  }
  Serial.println("Initialization finished"); //display a message

  if (USER_CALIBRATION) //calibrate the sensor
  {
    ToF_calibrate();                            //calibrate the sensor
    Serial.println("The sensor is calibrated"); //display a message
  }
  else if (FACTORY_CALIBRATION) //resotre default calibration settings
  {
    EEPROM_read(EEPROM_factory);                             //read factory calibration data
    Serial.println("Factory calibration settings restored"); //display a message
  }
  else //load user calibration data
  {
    EEPROM_read(EEPROM_user);                             //read user calibration data
    Serial.println("User calibration settings restored"); //display a message
  }
  return;
}

/*------------------------------------------------------------------------*/

/*
  starts a measurement
  arguments: nothing
  returns: nothing
*/
void ToF_start(void)
{
  //setup single shot mode
  write_reg(0x13, 0x7D); //load the register

  //enable interrupt
  write_reg(0x60, 0x01); //load the register

  //read clears interrupt
  read_reg(0x69); //address the register

  digitalWrite(SS, LOW);    //initiate a measurement
  delayMicroseconds(5600);  //wait 5.6 ms
  digitalWrite(SS, HIGH);   //reset pin state
  delayMicroseconds(14400); //wait 14.4 ms
  return;
}

/*------------------------------------------------------------------------*/

/*
  displays the measured distance on the serial monitor
  arguments: nothing
  returns: the distance
*/
float ToF_getMeasurement(void)
{
  ToF_start(); //initialize measurement
  while (digitalRead(IRQ) != 0)
    ; //wait for data

  unsigned char MSB = read_reg(0xD1);
  unsigned char LSB = read_reg(0xD2);

  float distance = (((double)MSB * 256 + (double)LSB) / 65536) * 3331; //convert measured data

  // Apply calibration offset
  distance -= CALIBRATION_OFFSET;

  //convert the result if required
#if defined(INCH)
  distance /= 2.54; //convert to inch
#elif defined(MM)
  distance *= 10; //convert to mm
#endif

  return distance;
}

/*------------------------------------------------------------------------*/

/*
  displays the measurement unit on the serial monitor
  arguments: none
  returns: none
*/
void display_unit(void)
{
  //inch
#ifdef INCH
  Serial.println(" inch"); //dispay message
  return;
#endif

  //cm
#ifdef CM
  Serial.println(" cm"); //dispay message
  return;
#endif

  //mm
#ifdef MM
  Serial.println(" mm"); //dispay message
  return;
#endif
}

/*------------------------------------------------------------------------*/

/*
  calibrates the Pmod ToF
  arguments: none
  returns: none
*/
void ToF_calibrate(void)
{
  //magnitude calibration: no user setup is needed
  Serial.println("Starting magnitude calibration... You have 5 sec to prepare the device"); //display a message
  Serial.println("No user setup is needed");                                                //display a message
  delay(5000);                                                                              //wait 5s
  ToF_calibrate_magnitude();                                                                //performe magnitude calibration

  //crosstalk calibration: block all light to the PD
  Serial.println("Starting crosstalk calibration... You have 10 sec to prepare the device"); //display a message
  Serial.println("Block all light towards the photodiode");                                  //display a message
  delay(10000);                                                                              //wait 10s
  ToF_calibrate_crosstalk();                                                                 //performe crosstalk calibration

  //distance calibration: mount the board to a known distance from the target
  Serial.println("Starting distance calibration...  You have 10 sec to prepare the device"); //display a message
  Serial.println("Mount the Pmod to a known distance from the target");                      //display a message
  delay(10000);                                                                              //wait 10s
  ToF_calibrate_distance(CALIBRATION_DISTANCE);                                              //perform distance calibration

  //save calibration values in EEPROM
  Serial.println("Saving calibration data"); //display a message
  EEPROM_write(EEPROM_user);                 //savevalues in EEPROM
  return;
}

/*------------------------------------------------------------------------*/

/*
  calibrates the signal magnitude, no user setup is needed
  arguments: none
  returns: none
*/
void ToF_calibrate_magnitude(void)
{
  //save interrupt control settings
  unsigned char interrupt_ctrl = read_reg(0x60); //save register value
  //save measurement settings
  unsigned char measurement_mode = read_reg(0x13); //save register value

  ToF_start(); //initiate a measurement
  while (digitalRead(IRQ) != 0)
    ; //wait for data

  //read-write magnitude exponent
  write_reg(cali_registers[8], read_reg(0xF6)); //load new register with read value
  //read-write magnitude MSB
  write_reg(cali_registers[9], read_reg(0xF7)); //load new register with read value
  //read-write magnitude LSB
  write_reg(cali_registers[10], read_reg(0xF8)); //load new register with read value

  //restore settings
  write_reg(0x60, interrupt_ctrl);   //restore interrupt settings
  write_reg(0x13, measurement_mode); //restore measurement settings
  return;
}

/*------------------------------------------------------------------------*/

/*
  calibrates the signal crosstalk, block all light towards the photodiode
  arguments: avg_nr - nr of measurements to average (default is 100)
  returns: none
*/
void ToF_calibrate_crosstalk(unsigned int avg_nr)
{
  //save interrupt control settings
  unsigned char interrupt_ctrl = read_reg(0x60); //save register value
  //save measurement settings
  unsigned char measurement_mode = read_reg(0x13); //save register value

  //average measured data
  double I = 0, Q = 0, G = 0;  //variables for measured data
  unsigned char exp, msb, lsb; //variables to read register into
  for (int i = 0; i < avg_nr; i++)
  {
    ToF_start(); //initiate a measurement
    while (digitalRead(IRQ) != 0)
      ; //wait for data

    exp += read_reg(0xDA);            //get exponent
    msb += read_reg(0xDB);            //get MSB
    lsb += read_reg(0xDC);            //get LSB
    I += bytes2double(exp, msb, lsb); //add current values to the sum

    exp += read_reg(0xDD);            //get exponent
    msb += read_reg(0xDE);            //get MSB
    lsb += read_reg(0xDF);            //get LSB
    Q += bytes2double(exp, msb, lsb); //add current values to the sum

    msb += read_reg(0xE6);   //get MSB
    lsb += read_reg(0xE7);   //get LSB
    G += ((msb << 8) | lsb); //add current values to the sum
  }
  I /= avg_nr; //average
  Q /= avg_nr; //average
  G /= avg_nr; //average

  unsigned char measured[13];                                //array for measurements
  double2bytes(I, &measured[0], &measured[1], &measured[2]); //get first 3 values
  double2bytes(Q, &measured[3], &measured[4], &measured[5]); //get second 3 values
  measured[6] = ((int)G & 0xFF00) >> 8;                      //get gain msb
  measured[7] = (int)G & 0xFF;                               //get gain lsb

  for (int i = 0; i < 8; i++)
  {
    measured[i] &= 0xFF;                       //truncate them if necessary
    write_reg(cali_registers[i], measured[i]); //load registers
  }

  //restore settings
  write_reg(0x60, interrupt_ctrl);   //restore interrupt settings
  write_reg(0x13, measurement_mode); //restore measurement settings
  return;
}

/*------------------------------------------------------------------------*/

/*
  calibrates the distance offset, mount the Pmod to a known distance from the target
  arguments: reference_distance - the reference distance in cm, avg_nr - nr of measurements to average (default is 100)
  returns: none
*/
void ToF_calibrate_distance(unsigned int reference_distance, unsigned int avg_nr)
{
  //save interrupt control settings
  unsigned char interrupt_ctrl = read_reg(0x60); //save register value
  //save measurement settings
  unsigned char measurement_mode = read_reg(0x13); //save register value

  int avg = 0;            //variable for averaging
  unsigned char MSB, LSB; //variables for data bytes
  for (int i = 0; i < avg_nr; i++)
  {
    ToF_start(); //initiate a measurement
    while (digitalRead(IRQ) != 0)
      ; //wait for data

    MSB = read_reg(0xD8);               //get phase MSB
    LSB = read_reg(0xD9);               //get phase LSB
    avg += ((int)MSB * 256) + (int)LSB; //add converted nr to sum
  }
  avg /= avg_nr; //calculate average

  //calculate the distance
  float distance = (float)avg - ((float)reference_distance * 1967.45722);

  MSB = ((unsigned int)distance & 0xFF00) >> 8; //get msb
  LSB = (unsigned int)distance & 0x00FF;        //get lsb
  write_reg(cali_registers[11], MSB);           //load MSB
  write_reg(cali_registers[12], LSB);           //load LSB

  //restore settings
  write_reg(0x60, interrupt_ctrl);   //restore interrupt settings
  write_reg(0x13, measurement_mode); //restore measurement settings
  return;
}

/*------------------------------------------------------------------------*/

/*
  read a register
  arguments: reg - register address
  returns: register value
*/
unsigned char read_reg(unsigned char reg)
{
  unsigned char val = 0x00;            //variable for values
  Wire.beginTransmission(i2c_address); //add the address to the buffer
  Wire.write(reg);                     //append register address
  Wire.endTransmission();              //send buffer
  Wire.requestFrom(i2c_address, 1);    //request 1 byte of data
  if (Wire.available())                //if data is available
  {
    val = Wire.read(); //read data byte
  }
  return val;
}

/*------------------------------------------------------------------------*/

/*
  write a register
  arguments: reg - register address, val - register value
  returns: none
*/
void write_reg(unsigned char reg, unsigned char val)
{
  Wire.beginTransmission(i2c_address); //add the address to the buffer
  Wire.write(reg);                     //append register address
  Wire.write(val);                     //set register state
  Wire.endTransmission();              //send buffer
  return;
}

/*------------------------------------------------------------------------*/

/*
  convert a double to three bytes
  arguments: nr - double, exp - exponent, msb - mantissa msb, lsb - mantissa lsb
  returns: none
*/
void double2bytes(double nr, unsigned char *exp, unsigned char *msb, unsigned char *lsb)
{
  bool negative = false; //check negativity
  if (nr < 0)
  {
    negative = true; //store flag
    nr = -nr;        //calculate absolute value
  }

  int e;
  double mantissa = frexp(nr, &e); //get the exponent
  *exp = e & 0xFF;                 //save the exponent
  for (int i = 0; i < 15; i++)
  {
    mantissa *= 2; //shift the mantissa 14 places to the left
  }

  if (negative)
  {
    mantissa = -mantissa; //make it negative
  }

  *msb = ((int)mantissa & 0xFF00) >> 8; //save msb
  *lsb = (int)mantissa & 0x00FF;        //save lsb
  return;
}

/*------------------------------------------------------------------------*/

/*
  convert three bytes to double
  arguments: exp - exponent, msb - mantissa msb, lsb - mantissa lsb
  returns: the converted number
*/
double bytes2double(unsigned char exp, unsigned char msb, unsigned char lsb)
{
  bool negative = false; //flag to signal negativity
  if (msb > 127)         //check if the number is negative or not
  {
    negative = true; // negative number
  }

  int mantissa = msb << 8; //recreate mantissa
  mantissa |= lsb;         //append lsb

  double result = 0; //variable to store the result
  if (negative)
  {
    mantissa = ((mantissa - 1) ^ 0xFFFF); // convert from 2's complement
    result = -mantissa * pow(2, exp);     // combine mantissa and exponent
  }
  else
  {
    result = mantissa * pow(2, exp); // combine mantissa and exponent
  }

  return result; //return the result
}

/*------------------------------------------------------------------------*/

/*
  displays all registers in the Serial Monitor
  arguments: none
  returns: none
*/
void debug(void)
{
  Serial.println('\n'); //leave a line out
  char str[5];          //string for output data

  //control, settings and status registers
  Serial.println("control, settings and status registers:"); //print message
  for (int i = 0x00; i <= 0x02; i++)
  {
    sprintf(str, "0x%2X", i);           //format data
    Serial.print(str);                  //print register address
    Serial.print("\t-\t");              //print separator
    sprintf(str, "0x%2X", read_reg(i)); //format data
    Serial.println(str);                //print register value
  }

  //sampling control registers
  Serial.println("sampling control registers:"); //print message
  for (int i = 0x10; i <= 0x13; i++)
  {
    sprintf(str, "0x%2X", i);           //format data
    Serial.print(str);                  //print register address
    Serial.print("\t-\t");              //print separator
    sprintf(str, "0x%2X", read_reg(i)); //format data
    Serial.println(str);                //print register value
  }
  Serial.print("0x19");                  //print register address
  Serial.print("\t-\t");                 //print separator
  sprintf(str, "0x%2X", read_reg(0x19)); //format data
  Serial.println(str);                   //print register value

  //closed loop calibration registers
  Serial.println("closed loop calibration registers:"); //print message
  for (int i = 0x24; i <= 0x30; i++)
  {
    sprintf(str, "0x%2X", i);           //format data
    Serial.print(str);                  //print register address
    Serial.print("\t-\t");              //print separator
    sprintf(str, "0x%2X", read_reg(i)); //format data
    Serial.println(str);                //print register value
  }

  //ambient light and temperature correction registers
  Serial.println("ambient light and temperature correction registers:"); //print message
  for (int i = 0x31; i <= 0x30; i++)
  {
    if (i == 0x31 || i == 0x33 || i == 0x34 || i == 0x36 || i == 0x39 || i == 0x3B) //only valid registers
    {
      sprintf(str, "0x%2X", i);           //format data
      Serial.print(str);                  //print register address
      Serial.print("\t-\t");              //print separator
      sprintf(str, "0x%2X", read_reg(i)); //format data
      Serial.println(str);                //print register value
    }
  }

  //interrupt registers
  Serial.println("interrupt registers:"); //print message
  Serial.print("0x60");                   //print register address
  Serial.print("\t-\t");                  //print separator
  sprintf(str, "0x%2X", read_reg(0x60));  //format data
  Serial.println(str);                    //print register value

  //analog control registers
  Serial.println("analog control registers:"); //print message
  for (int i = 0x90; i <= 0x93; i++)
  {
    sprintf(str, "0x%2X", i);           //format data
    Serial.print(str);                  //print register address
    Serial.print("\t-\t");              //print separator
    sprintf(str, "0x%2X", read_reg(i)); //format data
    Serial.println(str);                //print register value
  }
  Serial.print("0xA5");                  //print register address
  Serial.print("\t-\t");                 //print separator
  sprintf(str, "0x%2X", read_reg(0xA5)); //format data
  Serial.println(str);                   //print register value
  Serial.print("0xB0");                  //print register address
  Serial.print("\t-\\t");                //print separator
  sprintf(str, "0x%2X", read_reg(0xB0)); //format data
  Serial.println(str);                   //print register value

  //output registers
  Serial.println("output registers:"); //print message
  for (int i = 0xD1; i <= 0xE7; i++)
  {
    sprintf(str, "0x%2X", i);           //format data
    Serial.print(str);                  //print register address
    Serial.print("\t-\t");              //print separator
    sprintf(str, "0x%2X", read_reg(i)); //format data
    Serial.println(str);                //print register value
  }

  Serial.println('\n'); //leave a line out
  return;
}

/*------------------------------------------------------------------------*/

/*
  read data from EEPROM to cali_values array
  arguments: address - starting address
  returns: none
*/
void EEPROM_read(unsigned char address)
{
  Wire.beginTransmission(i2c_EEPROM); //add the address to the buffer
  Wire.write(address + 1);            //append data address
  Wire.endTransmission();             //send buffer

  for (int i = 0; i < 13; i++) //repeat for all bytes
  {
    Wire.requestFrom(i2c_EEPROM, 1); //request 1 byte
    if (Wire.available())            //if data is available
    {
      write_reg(cali_registers[i], Wire.read()); //read and save data byte
    }
  }
  return;
}

/*------------------------------------------------------------------------*/

/*
  write data to EEPROM from cali_values array
  arguments: address - starting address
  returns: none
*/
void EEPROM_write(unsigned char address)
{
  unsigned char data[13];      //array to store register values
  for (int i = 0; i < 13; i++) //read all registers
  {
    data[i] = read_reg(cali_registers[i]); //read calibration register value
  }

  Wire.beginTransmission(i2c_EEPROM); //add the address to the buffer
  Wire.write(address + 1);            //append data address
  for (int i = 0; i < 13; i++)        //go through the data array
  {
    Wire.write(data[i]); //append data byte
  }
  Wire.endTransmission(); //send buffer
  return;
}
